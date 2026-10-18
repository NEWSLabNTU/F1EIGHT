"""The plant's clock-handshake scheduler (lockstep_sync), without ROS."""
from autosdv_rc_sim.lockstep_sync import ACTUATOR, LockstepSync, VELOCITY_REPORT

MS = 1_000_000


def drive(sync, until_ns, batch_ns=5 * MS, participants=(), late=None):
    """Simulate the plant loop and ideal participants; return the /clock values published.

    participants: (name, period_ns, phase_ns); each acks every deadline the clock reaches,
    except those listed in ``late`` ({name: deadline_ns}), which it never acknowledges.
    """
    late = late or {}
    clock, published = 0, []
    nxt = {name: phase for name, _, phase in participants}
    while clock < until_ns:
        limit = sync.limit(clock)
        if limit is not None and limit <= clock:
            return published           # blocked: a participant owes an ack
        target = clock + batch_ns if limit is None else min(clock + batch_ns, limit)
        clock = target
        published.append(clock)
        for name, period, _ in participants:   # velocity_report first, as the actuator's gate does
            while nxt[name] <= clock:
                if late.get(name) == nxt[name]:
                    break
                sync.on_ack(name, nxt[name], period)
                nxt[name] += period
    return published


def test_no_participant_no_limit():
    sync = LockstepSync()
    assert sync.limit(123) is None
    assert drive(sync, 50 * MS) == list(range(5 * MS, 55 * MS, 5 * MS))


def test_learns_period_and_phase_from_the_first_ack():
    sync = LockstepSync()
    sync.on_ack(ACTUATOR, 20 * MS, 10 * MS)
    assert sync.limit(20 * MS) == 30 * MS
    assert sync.limit(25 * MS) == 30 * MS
    assert sync.limit(30 * MS) == 30 * MS          # at the deadline: wait for its ack
    assert sync.blocking(30 * MS) == [ACTUATOR]
    sync.on_ack(ACTUATOR, 30 * MS, 10 * MS)
    assert sync.limit(30 * MS) == 40 * MS


def test_clock_lands_on_every_deadline_and_never_passes_an_unacked_one():
    sync = LockstepSync()
    sync.on_ack(VELOCITY_REPORT, 0, 50 * MS)
    sync.on_ack(ACTUATOR, 3 * MS, 10 * MS)          # an off-batch phase
    clocks = drive(sync, 200 * MS, participants=[(VELOCITY_REPORT, 50 * MS, 50 * MS),
                                                 (ACTUATOR, 10 * MS, 13 * MS)])
    assert clocks == sorted(clocks)
    for d in range(13 * MS, 200 * MS, 10 * MS):
        assert d in clocks, d
    for d in range(50 * MS, 200 * MS, 50 * MS):
        assert d in clocks, d


def test_holds_at_a_missing_ack():
    sync = LockstepSync()
    sync.on_ack(VELOCITY_REPORT, 0, 50 * MS)
    sync.on_ack(ACTUATOR, 0, 10 * MS)
    clocks = drive(sync, 500 * MS, participants=[(VELOCITY_REPORT, 50 * MS, 50 * MS),
                                                 (ACTUATOR, 10 * MS, 10 * MS)],
                   late={ACTUATOR: 70 * MS})
    assert clocks[-1] == 70 * MS                    # reached the deadline, not past it
    assert sync.blocking(70 * MS) == [ACTUATOR]


def test_equal_instants_list_velocity_report_first():
    sync = LockstepSync()
    sync.on_ack(ACTUATOR, 0, 10 * MS)
    sync.on_ack(VELOCITY_REPORT, 0, 50 * MS)
    sync.on_ack('zz_other', 0, 50 * MS)
    assert sync.blocking(50 * MS) == [VELOCITY_REPORT, ACTUATOR, 'zz_other']


def test_timeout_invalidates_and_stops_waiting():
    sync = LockstepSync(timeout=2.0)
    sync.on_ack(VELOCITY_REPORT, 0, 50 * MS)
    sync.on_ack(ACTUATOR, 0, 10 * MS)
    clock = 10 * MS
    assert sync.check_timeouts(clock, wall=100.0) == []      # starts the wait clock
    assert sync.check_timeouts(clock, wall=101.9) == []
    assert sync.ok
    assert sync.check_timeouts(clock, wall=102.1) == [ACTUATOR]
    assert not sync.ok and 'actuator' in sync.reason and '0.010' in sync.reason
    assert sync.limit(clock) == 50 * MS                      # only velocity_report now
    # it comes back: waited for again, but the run stays invalid
    sync.on_ack(ACTUATOR, 60 * MS, 10 * MS)
    assert sync.limit(60 * MS) == 50 * MS
    assert sync.participants[ACTUATOR].live and not sync.ok


def test_an_ack_resets_the_wait_clock():
    sync = LockstepSync(timeout=2.0)
    sync.on_ack(ACTUATOR, 0, 10 * MS)
    sync.check_timeouts(10 * MS, wall=0.0)
    sync.on_ack(ACTUATOR, 10 * MS, 10 * MS)
    sync.check_timeouts(20 * MS, wall=1.9)
    assert sync.check_timeouts(20 * MS, wall=3.0) == []      # 1.1 s into the second wait
    assert sync.ok


def test_late_joiner_is_waited_for_at_its_current_deadline():
    sync = LockstepSync()
    sync.on_ack(ACTUATOR, 0, 10 * MS)
    # the clock got to 47 ms without it (e.g. after a timeout): its timer fires for 40 ms
    assert sync.participants[ACTUATOR].pending_ns(47 * MS) == 40 * MS
    assert sync.limit(47 * MS) == 40 * MS
    sync.on_ack(ACTUATOR, 40 * MS, 10 * MS)
    assert sync.limit(47 * MS) == 50 * MS


def test_reordered_or_duplicate_acks_are_ignored():
    sync = LockstepSync()
    sync.on_ack(ACTUATOR, 30 * MS, 10 * MS)
    sync.on_ack(ACTUATOR, 20 * MS, 10 * MS)
    assert sync.participants[ACTUATOR].last_ns == 30 * MS
    assert sync.participants[ACTUATOR].acks == 1


def test_stale_report_accounting():
    sync = LockstepSync()
    sync.on_ack(VELOCITY_REPORT, 50 * MS, 50 * MS)
    sync.on_ack(ACTUATOR, 60 * MS, 10 * MS, {'report': 0})        # gate not armed: apart
    assert sync.stale_reports == 0 and sync.ungated_ticks == 1
    sync.on_ack(VELOCITY_REPORT, 100 * MS, 50 * MS)
    sync.on_ack(ACTUATOR, 100 * MS, 10 * MS, {'report': 100 * MS, 'gate': 1})       # fresh
    sync.on_ack(ACTUATOR, 140 * MS, 10 * MS, {'report': 100 * MS, 'gate': 1})       # 40 ms old
    assert sync.stale_reports == 0
    sync.on_ack(VELOCITY_REPORT, 150 * MS, 50 * MS)
    sync.on_ack(ACTUATOR, 150 * MS, 10 * MS, {'report': 100 * MS, 'gate': 1})   # missed 150
    assert sync.stale_reports == 1
    assert sync.max_report_age_ns == 50 * MS
    sync.on_ack(ACTUATOR, 160 * MS, 10 * MS, {'skipped': 1})
    assert sync.skipped_ticks == 1
    s = sync.summary()
    assert s['stale_reports'] == 1 and abs(s['max_report_age'] - 0.05) < 1e-12
    assert s['participants'][ACTUATOR]['period'] == 0.01


def test_late_edges_are_reported():
    sync = LockstepSync()
    sync.on_ack(VELOCITY_REPORT, 0, 50 * MS, {'late_edges': 3})
    assert sync.summary()['late_edges'] == 3


def test_stale_command_accounting():
    from autosdv_rc_sim.lockstep_sync import PATH_TRACKER
    sync = LockstepSync()
    sync.on_ack(PATH_TRACKER, 0, 33_333_333)
    sync.on_ack(ACTUATOR, 30 * MS, 10 * MS, {'cmd': 0, 'cgate': 1})              # fresh
    sync.on_ack(ACTUATOR, 40 * MS, 10 * MS, {'cmd': 0, 'cgate': 1})              # missed 33.3
    sync.on_ack(ACTUATOR, 50 * MS, 10 * MS, {'cmd': 0, 'cgate': 0})              # gate off: apart
    assert sync.stale_commands == 1
    assert sync.summary()['stale_commands'] == 1
