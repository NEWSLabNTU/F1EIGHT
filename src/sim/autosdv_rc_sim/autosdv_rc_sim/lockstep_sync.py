"""The plant's side of the clock handshake: which /clock values it may publish. No ROS.

The speed loop runs in separate processes on the plant's /clock:

    plant --edges, /clock--> velocity_report (20 Hz) --report--> actuator (100 Hz) --PWM--> plant

Without a handshake a loaded host lets /clock run ahead of them: a tick is skipped, or the
actuator computes on an old report. With it, every participant acknowledges each timer
deadline on /rc_sim/lockstep/ack once its work for that deadline is done (hal.format_ack:
"<name> <sec> <nanosec> <period_ns> [key=value ...]"), and the plant never publishes a /clock
past a deadline it has no ack for. Simulated time then slows under load (real-time factor
below 1) instead of leaving the controller behind.

A participant is learned from its acks, nothing is configured: its first ack gives its phase
and its period. Its pending deadline is the next one after its last ack, or, if /clock is
already past that (it joined late, or came back after a timeout), the latest one at or before
/clock, which its timer is about to fire for.

    limit(clock)  the furthest /clock the plant may publish: the earliest pending deadline.
                  At or below the clock already published, the plant waits.

At equal instants velocity_report goes first: both nodes see the instant at once, so the
order is kept by the actuator, which holds a tick until the report of the latest
velocity_report deadline at or before it has arrived (hal.ReportGate), and acknowledges only
then. The plant checks it: each actuator ack carries the stamp of the report it used
(report=<ns>) and whether its gate was armed (gate=1: it knew velocity_report's schedule), and
a report older than the latest velocity_report deadline counts as stale. Ticks before the gate
is armed (the first acks, while DDS discovers the actuator's ack subscription) are counted
apart, as ungated.

The same holds one step earlier, between the plant and velocity_report: the edges of a batch
are published before its /clock, but on another topic, so velocity_report's timer can fire
before they are in. Under lockstep each edge message is marked complete up to the /clock that
follows (hal.SimGPIO), and velocity_report holds a report until its edges are complete. Its
acks carry late_edges=<n>, edges that still arrived after a report covering them (0 expected).

Lab 2 Part 1 adds path_tracker, the target's source: the actuator also holds a tick until the
control command stamped with the tracker's latest deadline at or before it has come through
the forwarder and vehicle_cmd_gate (both keep the stamp), and says so in its ack (cmd=<ns>,
cgate=1); an older one counts in stale_commands. The tracker's 30 Hz deadlines are off the
plant's 1 ms step, so the plant rounds a limit up to the next step (lands at most 1 ms past
the deadline, before the ack; the physics never runs ahead of /clock).

It also adds the e-stop's source, participant "emergency": lab2_part1_scenario's 20 Hz
emergency timer, which applies a scheduled e-stop change at its tick, stamps the message with
the tick's deadline and acknowledges after publishing. The actuator's third gate holds a tick
until the emergency message of the scenario's latest tick at or before it has arrived, so the
e-stop acts on the same actuator tick in every run (egate=1 in its acks).

The actuator's command and emergency gates arm only on a message stamped on their
participant's schedule (one physics step after a deadline at most), never on one some other
publisher made up, and drop out after 0.5 s (wall) without the message they wait for; the
next on-schedule message re-arms them. On a stack where nobody acknowledges as path_tracker
or emergency (just sim rc, the car) they never arm.

A participant that leaves a pending deadline unacknowledged for `timeout` wall seconds times
out: the run is invalid from then on (`ok` false, the reason kept), and the plant stops
waiting for it until it acknowledges again, so a dead node never hangs the simulation.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# the participants' names: the one definition is hal's (it imports no ROS)
from autosdv_vehicle_interface.hal import (  # noqa: F401  (EMERGENCY re-exported)
    ACTUATOR, EMERGENCY, PATH_TRACKER, VELOCITY_REPORT)


@dataclass
class Participant:
    name: str
    period_ns: int
    last_ns: int                        # the last acknowledged deadline
    live: bool = True                   # False after a timeout, until it acknowledges again
    blocked_since: Optional[float] = None   # wall time the plant started waiting for it
    acks: int = 0
    extra: Dict[str, int] = field(default_factory=dict)   # the last ack's key=value fields

    def pending_ns(self, clock_ns: int) -> int:
        """The deadline this participant must acknowledge next, given the published clock."""
        missed = (clock_ns - self.last_ns) // self.period_ns
        return self.last_ns + self.period_ns * max(1, missed)


class LockstepSync:
    def __init__(self, timeout: float = 2.0):
        self.timeout = float(timeout)
        self.participants: Dict[str, Participant] = {}
        self.ok = True
        self.reason = ''
        self.timeouts = 0
        self.stale_reports = 0          # actuator ticks on a report older than they should be
        self.skipped_ticks = 0          # actuator ticks acknowledged without running (timeout)
        self.ungated_ticks = 0          # actuator ticks before it knew velocity_report's schedule
        self.max_report_age_ns = 0      # actuator tick time - stamp of the report it used
        self.late_edges = 0             # edges velocity_report got after a report covering them
        self.stale_commands = 0         # gated ticks on a command older than the tracker's

    # ------------------------------------------------------------ acks
    def on_ack(self, name: str, deadline_ns: int, period_ns: int,
               extra: Optional[Dict[str, int]] = None) -> None:
        extra = dict(extra or {})
        p = self.participants.get(name)
        if p is None:
            p = self.participants[name] = Participant(name, int(period_ns), int(deadline_ns))
        elif deadline_ns < p.last_ns:
            return                      # a duplicate or reordered ack: nothing new
        p.period_ns, p.last_ns = int(period_ns), int(deadline_ns)
        p.live, p.blocked_since, p.extra = True, None, extra
        p.acks += 1
        if name == VELOCITY_REPORT:
            self.late_edges = max(self.late_edges, extra.get('late_edges', 0))
        elif name == ACTUATOR:
            self._check_report(deadline_ns, extra)

    def _check_report(self, tick_ns: int, extra: Dict[str, int]) -> None:
        if extra.get('skipped'):
            self.skipped_ticks += 1
            return
        expected_cmd = self.expected_ns(PATH_TRACKER, tick_ns)
        if extra.get('cgate') and expected_cmd is not None and extra.get('cmd', -1) < expected_cmd:
            self.stale_commands += 1
        report_ns = extra.get('report')
        expected = self.expected_report_ns(tick_ns)
        if report_ns is None or expected is None:
            return
        if not extra.get('gate'):
            # the actuator does not know velocity_report's schedule yet (its ack subscription
            # is still being discovered), so it could not wait for the report
            self.ungated_ticks += 1
            return
        if report_ns < expected:
            self.stale_reports += 1
        if report_ns >= 0:
            self.max_report_age_ns = max(self.max_report_age_ns, tick_ns - report_ns)

    def expected_report_ns(self, tick_ns: int) -> Optional[int]:
        """The latest velocity_report deadline at or before ``tick_ns`` (None: not known yet)."""
        return self.expected_ns(VELOCITY_REPORT, tick_ns)

    def expected_ns(self, name: str, tick_ns: int) -> Optional[int]:
        """The participant's latest deadline at or before ``tick_ns`` (None: not known yet)."""
        p = self.participants.get(name)
        if p is None:
            return None
        return p.last_ns + ((tick_ns - p.last_ns) // p.period_ns) * p.period_ns

    # ------------------------------------------------------------ the clock
    def limit(self, clock_ns: int) -> Optional[int]:
        """The furthest /clock the plant may publish (None: no live participant, no limit)."""
        pending = [p.pending_ns(clock_ns) for p in self.participants.values() if p.live]
        return min(pending) if pending else None

    def blocking(self, clock_ns: int) -> List[str]:
        """The live participants the plant waits for at ``clock_ns``, velocity_report first."""
        names = [p.name for p in self.participants.values()
                 if p.live and p.pending_ns(clock_ns) <= clock_ns]
        return sorted(names, key=lambda n: (n != VELOCITY_REPORT, n != ACTUATOR, n))

    def check_timeouts(self, clock_ns: int, wall: float) -> List[str]:
        """Start the wait clock of each blocking participant; return those that timed out now."""
        out = []
        for name in self.blocking(clock_ns):
            p = self.participants[name]
            if p.blocked_since is None:
                p.blocked_since = wall
            elif wall - p.blocked_since > self.timeout:
                p.live, p.blocked_since = False, None
                self.timeouts += 1
                out.append(name)
                if self.ok:
                    self.ok = False
                    t = p.pending_ns(clock_ns) * 1e-9
                    self.reason = (f'{name} did not acknowledge t={t:.3f} s '
                                   f'within {self.timeout:g} s (wall)')
        return out

    def summary(self) -> dict:
        return {
            'participants': {p.name: {'period': p.period_ns * 1e-9, 'last': p.last_ns * 1e-9,
                                      'live': p.live, 'acks': p.acks}
                             for p in self.participants.values()},
            'timeouts': self.timeouts,
            'stale_reports': self.stale_reports,
            'skipped_ticks': self.skipped_ticks,
            'ungated_ticks': self.ungated_ticks,
            'max_report_age': self.max_report_age_ns * 1e-9,
            'late_edges': self.late_edges,
            'stale_commands': self.stale_commands,
        }
