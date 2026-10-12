# Coach-board pursuit stack: what runs and why

> This is the coach-board version of Lab 2, kept under `just coach`. Lab 2
> became ArUco localization and waypoint following on 2026-10-13; its stack is
> [lab2-waypoint-stack.md](lab2-waypoint-stack.md), and the system component,
> API component, control component and safety notes below are shared by both.

`just coach vehicle` (`coach_pursuit.launch.xml`) and `just coach sim`
(`coach_pursuit_sim.launch.xml`) start only the nodes the coach-board pursuit
lab uses; `just coach bench` (`coach_pursuit_bench.launch.xml`) runs the vehicle
launch on a workstation with the hardware replaced (see
[The vehicle bench](#the-vehicle-bench)). Everything else Autoware and AutoSDV would normally start is
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
| `launch_sensing_driver`, `launch_velocity_report` | `coach_pursuit.launch.xml` (the latter also `vehicle_interface.launch.xml`) | true on the car, false on the bench |

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
  (`launch_planner:=false`); `just coach speed-steps` then drives the car;
  disengage returns to STOP.
- `just coach vehicle launch_sensing:=false is_simulation:=true` starts on a
  workstation (velocity_report exits: it needs `Jetson.GPIO`), and
  `/system/operation_mode/availability` has exactly one publisher, the
  converter.

## The vehicle bench

`just coach sim` replaces the whole vehicle interface with
`simple_planning_simulator`, so the actuator's safety shell, its watchdogs, its
MRM braking, the student's SpeedPID plug-in and dead reckoning never run in the
loop there. `just coach bench` runs the **real** `coach_pursuit.launch.xml` on a
workstation and replaces only the hardware:

| On the car | On the bench |
|---|---|
| Robin-W driver, ZED camera node (`launch_sensing_driver:=false`) | nothing; the ZED IMU frame (`zedxm_imu_tf`) and `imu_corrector` stay |
| PCA9685 (the actuator writes it) | the actuator's dry run (`is_simulation:=true`); it still publishes `~/debug/pwm_values` every tick |
| `velocity_report` (`launch_velocity_report:=false`) | `vehicle_bench_plant` publishes `/vehicle/status/velocity_status` (20 Hz, `base_link`) |
| ZED IMU | `vehicle_bench_plant` publishes `/sensing/camera/zedxm/imu/data` in `zedxm_imu_link`, with the static gyro bias `imu_corrector.param.yaml` removes |
| `board_tracking` (`launch_detector:=false`) | `virtual_coach`, placed from the plant's true pose |

Everything else is the vehicle launch's 44-node set, unmodified: 41 nodes on
the bench. `vehicle_bench_plant` (`autosdv_coach_lab`) integrates the PWM the
actuator commands through the motor fit's `MotorPlant` (same
`MotorPlantParams`, so `plant:="$(cat tmp/motor_plant.yaml)"` loads a fitted
motor), the actuator's steering mapping inverted, bicycle kinematics about
`base_link`, and `HallSpeedSensor` (velocity_report's edge timing, EMA and
0.5 s timeout). Its constants come from the files the actuator and
velocity_report read. When the actuator stops publishing, the motor goes to
neutral after 0.5 s, because every actuator exit writes neutral but a dry run
does not publish that write (`on_pwm_loss:=hold` keeps the last PWM instead).

```bash
just coach bench                                   # default scenario, reference laws
just coach bench scenario:=dropout speed_controller_class:=my_lab2.speed_pid:SpeedPID
just coach engage                                  # record, engage, disengage as on the car
just coach bench-checks                            # protocol 3.1 checks 1-4, PASS/FAIL lines
```

`just coach bench-checks` (`scripts/testing/lab2/bench_checks.py`) starts a
fresh bench per check and drives it through the TA's recipes:

| Check | Protocol 3.1 | Passes when |
|---|---|---|
| `follow` | -- | engage succeeds on the first try; the default scenario is followed; `just coach record` bag graded by `ta/grade.py --runs` |
| `mrm` | 3 | killing `board_pursuit_planner` while following gives an emergency-stop MRM, `override_brake:emergency`, and a stop |
| `gate` | 1 | unloading `vehicle_cmd_gate` gives `override_brake:command_timeout` within 0.6 s, then `override_neutral`, and a stop |
| `exit` | 2 | the actuator's process-level test (`test_process_exit_leaves_outputs_neutral`): SIGINT and SIGTERM leave 370 / 489. A dry run does not publish the exit write, so the bench cannot observe it |
| `fault` | 4 | `autosdv_coach_lab.faulty_speed_controller:RaisingSpeedPID`: neutral, retries 1 s apart (from the actuator log; a retry faults inside the same tick, so `~/debug/mode` never leaves `controller_fault`), disabled after 3, node alive, PWM 370 |
| `speedpid` | -- | the course's reference SpeedPID drives the car; the handout's stub does not |

It uses `ROS_DOMAIN_ID` (default 73, 0 refused) and `LAB2_WEB_ADDR`, and stops
each bench by its session, so it is safe on a shared host. Logs go to
`tmp/lab2bench/checks/`.

### What the bench showed (2026-10-12, provisional motor parameters)

```
PASS  follow.engage: first attempt: success=True, code=0, message=''
PASS  follow.drive: drove 24.7 m (true pose), peak 0.95 m/s, min true gap 1.26 m
PASS  follow.grade: 451 rows, follow RMSE 0.17 m, min headway 0.83 m
WARN  follow.grade: min headway 0.83 m is below the 1.0 m floor (true minimum gap 1.26 m)
PASS  mrm: MRM after 1.16 s, behavior 2 (EMERGENCY_STOP); override_brake:emergency; stopped 1.5 s after the kill
PASS  gate: override_brake:command_timeout 0.33 s after the unload; then override_neutral; last PWM (370, 489)
PASS  exit: 3 passed
PASS  fault: 3 faults, retries after 1.00 s, 1.01 s; disabled after fault 3; actuator alive, PWM 100 Hz, motor 370
PASS  speedpid: reference 11.9 m / peak 1.50 m/s; stub 0.00 m / peak 0.00 m/s in 20 s
```

Two things the planning sim cannot show:

- **The real speed loop stops later.** Static-approach overshoot is 0.5 to
  0.6 m on the bench against 0.1 m in the sim (`ta/grade.py`, default
  scenario), and the true minimum gap at the board stop is 1.0 to 1.3 m. The
  actuator's default PID tracks the planner's command with a 0.26 m/s RMSE.
  Re-measure with the fitted motor (`plant:=`) before reading much into it.
- **Dead reckoning runs ahead at every stop.** velocity_report holds its last
  EMA speed for up to 0.5 s after the last marker edge, so dead reckoning ends
  0.4 m ahead of the true pose after the default scenario. `ta/grade.py`
  measures headway against dead reckoning, so on the car (as on the bench) its
  headway reads short by that much: 0.83 m graded against 1.26 m true.

## Preflight

`just coach preflight` (`scripts/testing/lab2/preflight.py`) runs against a
stack that is up -- `just coach vehicle`, `bench` or `sim` -- before the first
engage, and prints PASS / WARN / FAIL lines; it exits 1 on any FAIL.

| Check | What |
|---|---|
| nodes | the slim set for the mode is alive; exactly one trajectory publisher (`board_pursuit_planner` or `speed_step_planner`); on the car, a LiDAR driver |
| rates | LiDAR cloud >= 7 Hz (car), IMU >= 20 Hz (car, bench), `velocity_status` >= 10 Hz (car, bench), `/perception/coach/board` >= 8 Hz, `kinematic_state` >= 10 Hz; a WARN band below the nominal rate |
| tf | `base_link` -> the LiDAR frame (car; the frame comes from the cloud's header), `map` -> `base_link` |
| availability | `/system/operation_mode/availability` has exactly one publisher, `/system/converter` |
| mrm | `/system/fail_safe/mrm_state` is NORMAL |
| mode | operation mode STOP, not in transition |
| graph | `/autoware/modes/autonomous` is OK; otherwise the deepest non-OK units under it, i.e. what blocks engage |

The mode is detected from the node list (`simple_planning_simulator` -> sim,
`vehicle_bench_plant` -> bench, else vehicle); `--mode` forces it, and
`--no-board` turns a missing board into a WARN for before the board is placed.
On the car, the board check is the protocol's "board at 4 m, at least 8 Hz".
