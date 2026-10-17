# RC simulator: interactive-run fixes (Tasks 18b and 18c)

Development history for `just sim rc`, moved out of the student guide
(`docs/guides/rc-sim.md`). It records root causes and evidence, then the open items.

## Clock period

Raising `clock_period` from 0.005 to 0.01 to save `/clock` traffic was tried
twice and the interactive run crept at about 0.15 m/s both times. The cause
turned out to be the emergency flap below, which crept the unmodified stack too,
so that rejection was not about `clock_period`; the lever was not re-measured and
stays at the default.


## The emergency flap (fixed) and the creep

Until Task 18b every `just sim rc` run, passing ones included, logged
`vehicle_cmd_gate: Emergency!` about once a second and `mrm_handler` toggled
EMERGENCY_STOP for 0.1 s each time, braking the actuator. The cause: the
diagnostic graph (`autoware-main.yaml`) gives `/autoware/localization/state`,
`/autoware/planning/routing/state` and `/adapi/mrm_request/delegate` the default
1.0 s timeout, and the `/adapi` nodes publish them every 1.0 s. On the plant's
simulated clock each publish is stamped on a whole second and the aggregator's
10 Hz tick falls on the same simulated instant, so every second the tick reads
the previous stamp at exactly age 1.0 s before the new one arrives, the unit
reads STALE for one tick, `autonomous` drops out of the graph and the MRM
handler stops the car. Upstream's CARLA graph (`autoware-carla.yaml`) sets
`timeout: 3.0` on `localization/state` and the MRM delegate too, but drops
`routing/state` altogether, and its stated reason is network latency and slow
update rates, not tick alignment: a loose precedent for the remedy, not for the
cause. rc_sim now passes its own graph,
`config/diagnostics/rc_sim_diagnostics.yaml` (autoware-main.yaml with the three
timeouts at 3.0; `localization.yaml` and `planning.yaml` are copies of
upstream's with the one line added; re-diff after an Autoware upgrade). The car's
own launch is untouched. After the change a run shows no MRM state change and no
loss of autonomous availability, and `smoke.py` asserts both (before: 48 MRM
changes and 46 availability drops in one run; after: 0 and 0, three runs of
three).

The creep, the car engaged but reaching only 0.15 m/s, was the flap, which
reproduced it on demand (`vmax` 0.14 on the first run of this task with the old
stack and default coefficients).

Whether the stock `just sim planning` (wall clock) has the same flap was not
measured. The lock needs the publisher and the aggregator on one stepped
simulated clock, so each 1 Hz stamp and each 10 Hz tick fall on the same
instant; on the wall clock their phases drift apart. Treat the 3.0 s timeouts
as specific to rc_sim's clock until someone runs the planning simulator
with the diagnostics logged.

## The stop overshoot (fixed): the hall speed saturated

With the flap gone the car overshot the stop line by 8-22 m (4-5 m with
`integral_limit` 60), although the grader stops the same controller within
0.5 m. It was not integrator windup. Recorded interactive run
(`/control/command/control_cmd`, `/vehicle/status/velocity_status`, the true
speed, `/rc_sim/pwm`): Autoware's profile matched the graded one (peak 11 m/s,
falling at 2.5 m/s^2), but `velocity_status` read at most about 8 m/s while the
car ran at 12, so the PID, seeing a car 4-6 m/s too slow, held PWM at 470
through the braking.

`VelocityEstimator` caps its estimate at `1.5 * marker_spacing / (now - last
edge)`; at 0.0275 m per marker that is 8.3 m/s when the last edge is 5 ms old.
The plant published `/clock` for a batch before the edges of that batch, so at
speed the clock always led the newest edge it could see by up to `clock_period`
(5 ms) and the cap cut in. The car has no such gap (it times an edge on arrival
with the same clock), and the lockstep grader has none either.
`rc_sim_plant` now steps the batch, then publishes the edges, then `/clock`, then
the stamped data, so an edge is published before the tick that
covers it. The same run stops at x 105.38 (line 105.40) with the reference
coefficients (`integral_limit` 100), and `smoke.py` now uses exactly those
and requires a stop within 1 m. Residual: the order is a publication order,
not a delivery guarantee across topics; a loaded host could in principle
reorder it, and the symptom would be the same overshoot.

## Open items

- **Smoke run 3, control-diagnostics stall at load 11.** One smoke run in a series
  showed the control diagnostics stalling on a loaded host; it did not reproduce on
  a quiet one. Not explained; the smoke gate is the guard.
- **Edge/clock publish-order fallback.** The edges-then-clock order is a publication
  order, not a delivery guarantee across topics. If a loaded host reorders it, the
  fallback is to lag `/clock` by one batch, so every edge of a batch is older than
  the clock that covers it.
- **`just sim planning` (wall clock) is unmeasured** for the emergency flap; see the
  note above. Treat the 3.0 s timeouts as specific to rc_sim's clock until someone
  runs the planning simulator with the diagnostics logged.
