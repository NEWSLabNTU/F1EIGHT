# Diagnostic graph configs

The diagnostic graph decides whether `/autoware/modes/autonomous` is
*available*. If any branch is ERROR or STALE, `change_to_autonomous` is refused
with "The target mode is not available", no matter how good localization is.

## Which files in this directory are actually loaded

Only these two, and only for `pose_source:=mcl`:

| File | Loaded when |
|---|---|
| `autosdv-mcl-main.yaml` | `pose_source:=mcl` (top-level graph) |
| `map-mcl.yaml` | included by the above |
| `localization-mcl.yaml` | included by the above |

And, for the coach-pursuit lab (Lab 2), selected explicitly by
`coach_pursuit.launch.xml` and `coach_pursuit_sim.launch.xml`:

| File | Loaded when |
|---|---|
| `autosdv-pursuit-main.yaml` | coach pursuit (top-level graph) |
| `control-pursuit.yaml`, `localization-pursuit.yaml`, `perception-pursuit.yaml`, `planning-pursuit.yaml` | included by the above |

The pursuit graph has no map, no route, no NDT/EKF and no Autoware perception
or planning checks; it gates autonomous on dead-reckoned odometry, the
pursuit planner's trajectory rate, coach_tracker being alive, and the stock
control (minus lane departure and AEB), vehicle and system checks. Local and
remote (manual) modes are `type: error`: the lab runs no
external_cmd_converter. See `docs/guides/lab2-stack.md`. Its topic
monitors are `../component_state_monitor/topics-pursuit.yaml`. The board
detection rate is reported as `/autoware/lab/coach_detection` but gates
nothing: a lost board is the planner's stop, not an MRM.

Every other `.yaml` here — `autoware-main.yaml`, `autoware-awsim.yaml`,
`control.yaml`, `hardware.yaml`, `localization.yaml`, `map.yaml`,
`perception.yaml`, `planning.yaml`, `system.yaml`, `vehicle.yaml` — is **not
loaded by anything**. For every non-`mcl` pose source the graph comes from the
installed `autoware_launch` package.

Those files are also *stale*: they were copied from an older Autoware and still
name pre-1.5.0 diagnostic nodes (`localization.yaml` sources
`/autoware/localization/state` from `component_state_diagnostics`, where 1.5.0
uses `/adapi/node/localization`). Editing them has no effect, and making them
live as-is would introduce new STALE checks. They are kept only because
deleting config is a separate decision from fixing the health checks; treat
them as historical.

The dead-config situation existed because `autosdv_autoware.launch.xml`
included `tier4_system_component` with no arguments, so it resolved its own
defaults. Two intended AutoSDV edits therefore never applied: disabling the
localization accuracy check, and correcting the monitored trajectory topic to
`/planning/scenario_planning/trajectory`. Only the second was carried into the
`mcl` variants; the accuracy check is deliberately left enabled, because
turning it off was never actually in effect and doing so now would be a new,
unmeasured change to emergency behaviour.

## Why `mcl` needs its own graph

The stock graph encodes "localization means NDT against a PCD map":

- `map.yaml` requires `/map/pointcloud_map`. Under `mcl` there is no PCD —
  `autosdv_map_component` deliberately does not start `pointcloud_map_loader` —
  so the check can never pass.
- `localization.yaml` sources `scan_matching_status` from `ndt_scan_matcher`,
  which does not exist under `mcl`, so it stays STALE forever.

Measured before the fix, on the sample site with an otherwise healthy stack
(localization INITIALIZED, route SET): `change_to_autonomous` refused, with
both of the above named among the aggregator's reasons. After the fix, neither
`/autoware/map` nor any NDT-specific localization check appears in the reason
list.

`map-mcl.yaml` requires the 2-D occupancy grid instead, monitored via
`../component_state_monitor/topics-mcl.yaml` (topic `/map`, latched, so it is a
presence check exactly like the stock map entries).

## Known gap

Under `mcl` the graph has **no estimator-health signal**. The NDT check was
dropped rather than faked, because `particle_filter` publishes no ROS
diagnostic today — its `diagnostics.py` writes a JSONL trace, not
`diagnostic_msgs`. A silently diverging filter will not raise a diagnostic
here. Closing this means publishing a real health diagnostic (effective sample
size and mean particle weight are already computed per update) and adding it to
`localization-mcl.yaml`.
