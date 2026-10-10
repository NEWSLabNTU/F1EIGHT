# Lab 2 stack: what runs and why

`just lab2 vehicle` (`coach_pursuit.launch.xml`) and `just lab2 sim`
(`coach_pursuit_sim.launch.xml`) start only the nodes the coach-board pursuit
lab uses. Everything else Autoware and AutoSDV would normally start is
switched off by launch arguments; the stock launches (`just launch`,
`just sim logging`) are unchanged.

| | vehicle | sim (headless) |
|---|---|---|
| nodes, before 2026-10-11 | 83 | 82 |
| nodes, now | 44 | 37 |
| processes under `ros2 launch` (containers + standalone) | 38 -> 25 | 33 -> 18 |
| CPU, whole stack, following (play_launch, this workstation) | -- | 72% -> 35% of one core |
| RSS, whole stack | -- | 2.7 GB -> 1.0 GB |

The vehicle numbers are from `play_launch resolve` only; nobody has measured
them on a Jetson yet. play_launch runs each composable node as its own
process, so under play_launch the process count is close to the node count.

## The pipeline

```
LiDAR driver -> board_tracking -> /perception/coach/board      (sim: virtual_coach)
  -> coach_tracker -> /perception/coach/state
  -> board_pursuit_planner -> /planning/trajectory               (speed steps: speed_step_planner)
  -> trajectory_follower (pure pursuit + PID) -> vehicle_cmd_gate -> /control/command/control_cmd
  -> actuator -> PWM                                             (sim: simple_planning_simulator)
velocity_report + IMU -> dead_reckon_odometry -> /localization/kinematic_state, map->base_link
```

## What runs

Everything below has a consumer, or is one of the checks that gate engage and
trigger an MRM.

**Lab nodes.** `board_tracking` (vehicle) or `virtual_coach` (sim),
`coach_tracker`, `board_pursuit_planner` (absent with `launch_planner:=false`).

**Sensing (vehicle).** The LiDAR driver (`seyond_node` for Robin-W, the Nebula
Velodyne node for `lidar_model:=vlp16`); the detector reads its cloud
directly. The ZED container and camera node for the IMU, and the ZED's
`robot_state_publisher`, which provides the IMU frame. `imu_corrector`, whose
`/sensing/imu/imu_data` feeds dead reckoning. (`mpu9250driver` instead of the
ZED with `imu_source:=mpu9250`.)

**Vehicle (vehicle).** `actuator` (brakes on `/control/command/emergency_cmd`
and on any MRM), `velocity_report` (wheel speed; dead reckoning and the
velocity rate check), `steering_status` (the steering rate check, and the
controllers' steering input), `control_mode_manager` (the control-mode report
the transition manager engages through), `gear_manager` (the gear report).

**Localization stand-in.** `dead_reckon_odometry` (`/localization/kinematic_state`
and `map->base_link`; in the sim the simulator publishes both) and
`twist2accel` (`/localization/acceleration`, which the gate and the controller
read). `robot_state_publisher` for `base_link -> sensor` frames; the detector
needs `base_link -> LiDAR`.

**Control** (`autosdv_pursuit_control_component`). In one container:
`trajectory_follower/controller_node_exe`, `vehicle_cmd_gate`,
`autoware_operation_mode_transition_manager`, and `external_cmd_selector`. The
selector looks like a manual-control part, but the transition manager will not
publish `/autoware/engage` until it has heard the selector's mode, so AUTONOMOUS
needs it.

**System** (`autosdv_pursuit_system_component`).

- `aggregator` and `converter`: `autosdv-pursuit-main.yaml` turned into
  `/system/operation_mode/availability`, read by engage and by the MRM handler.
- Ten `topic_state_monitor`s in `component_state_monitor/container`: the rate
  checks the graph is made of (trajectory, both control commands, emergency
  command, velocity, steering, `map->base_link`, kinematic state, coach state,
  and coach board, which is informational).
- `duplicated_node_checker`: linked into autonomous; two planners or two
  actuators refuse engage.
- `mrm_handler`, `mrm_emergency_stop_operator` (the gate's emergency command),
  `mrm_comfortable_stop_operator` (the handler's comfortable stop).
- `logging_diag_graph`: prints which check blocks autonomous.

**AD API** (`autosdv_pursuit_api_component`). `operation_mode` (engage,
disengage, `/api/operation_mode/state`), `mrm_request` (its `delegate`
diagnostic is linked into the autonomous and stop modes); in the sim also
`localization`, which `sim_pose_init` calls.

**Simulator only.** `simple_planning_simulator`, `pose_initializer`,
`sim_pose_init`, `drive_gear_hold`, and the map when RViz runs (it is the
backdrop and nothing else).

## What is off, and why

| Off | Why it is not needed |
|---|---|
| 16 of the 19 default AD API adapters, the RViz pose/route adaptors, the RTC controller | No lab node calls or reads them. `motion` paused the gate after 1 s at a standstill and released it as soon as the controller asked for speed, which only delayed restarts. |
| `processing_time_checker`, `pipeline_latency_monitor`, `service_log_checker`, `hazard_status_converter`, `component_state_monitor/component` | Nothing in the lab graph or the lab nodes reads them. The StateMonitor component's only reader was the ADAPI `autoware_state` node. |
| `external_cmd_converter` | Converts manual pedal commands for the gate's EXTERNAL input. Local and remote modes are marked unavailable in `control-pursuit.yaml`. |
| `control_validator` | Its diagnostics are in no lab graph unit. |
| `shift_decider` | It reads `/autoware/state`, which nothing publishes here, and with no route it never commanded DRIVE anyway. The gate sends gear NONE: the vehicle's actuator ignores gear, and in the sim `drive_gear_hold` owns the gear topic. |
| control check container, glog components, `pointcloud_container` | Hosted nothing the lab uses. |
| `passthrough_as_concatenate` | Builds `/sensing/lidar/concatenated/pointcloud`; the detector reads the driver's cloud. |
| `vehicle_velocity_converter` | Fed `gyro_odometer`. Dead reckoning reads `velocity_status` directly. |
| `gyro_bias_estimator` | Publishes a gyro bias diagnostic only; `imu_corrector` applies the configured bias. |
| `signal_manager` | Turn-indicator and hazard status; nothing reads it. |
| `empty_objects_publisher`, the simulator's perception modules (occupancy grid, `pointcloud_to_laserscan`, analytics publisher) | Served Autoware planning, which the lab replaces. |
| `dummy_diag_publisher` (sim) | None of the names it fakes is a unit in the lab graph. |
| the map (headless sim) | The simulator has road slope off, and the pose initializer does not fit heights in API mode. |

One change is a fix rather than a removal: `control_mode_manager` no longer
publishes `/system/operation_mode/availability` in the lab
(`publish_operation_mode_availability:=false`). The graph converter publishes
the same topic, and the manager's copy always says "autonomous available", so
with both running the MRM handler and the engage check read whichever arrived
last and a failed check could be overwritten. The stock vehicle launch still
has both publishers.

## Switches

All default to the previous behaviour, so only the lab launches change.

| Argument | Where | Lab value |
|---|---|---|
| `launch_system`, `launch_api` | `autosdv.launch.yaml` / `autosdv_autoware.launch.xml` | false (the lab components replace them) |
| `launch_shared_pointcloud_container` | same | false |
| `launch_empty_objects_publisher` | `autosdv.launch.yaml` | false |
| `launch_pointcloud_preprocessor`, `launch_vehicle_velocity_converter`, `launch_gyro_bias_estimator` | sensor kit (`lidar`, `sensing`, `imu` launch files) | false |
| `launch_signal_manager`, `publish_operation_mode_availability` | `vehicle_interface.launch.xml` | false |
| `launch_map` | `coach_pursuit_sim.launch.xml` | follows `rviz` |

The sensor-kit switches reach the kit only as launch configurations (`<let>`
in `coach_pursuit.launch.xml`), because the stock sensing component does not
forward them.

## Checked in the planning sim (2026-10-11)

- Engage succeeds; the reference controller gives the same metrics as before
  (`ta/grade.py --runs`): default overshoot 0.09 m (before 0.15), follow RMSE
  0.05 m (0.06), min headway 1.06 m (1.06); dropout and hardstop likewise
  within run-to-run noise.
- Detection loss (dropout) stops the car without an MRM.
- Killing `board_pursuit_planner` triggers an emergency-stop MRM 1.2 s after
  the last trajectory, and the car stops.
- Engage is refused while no trajectory is published
  (`launch_planner:=false`); `just lab2 speed-steps` then drives the car;
  disengage returns to STOP.
- `just lab2 vehicle launch_sensing:=false is_simulation:=true` starts on a
  workstation (velocity_report exits: it needs `Jetson.GPIO`), and
  `/system/operation_mode/availability` has exactly one publisher, the
  converter.
