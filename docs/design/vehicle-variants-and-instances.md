# Vehicle variants and instances: how a team configures its own cars

Status: implemented (2026-10-19).

## The problem

AutoSDV is the base project other teams build on, and every team's cars differ:
the sensors on the roof, the chassis, and, between two copies of the same car,
the numbers measured on that one copy (a stop PWM, a LiDAR's mounting yaw). Our
own fleet already shows all three:

| | sensors | chassis | measured on that copy |
|---|---|---|---|
| car A | VLP16 + ZED X Mini | AutoSDV RC | neutral 370, steer centre 489 |
| car B | Robin-W + ZED X Mini | AutoSDV RC | neutral 395, steer centre 400, ratio -132 |
| car C | Robin-W + ZED X Mini | AutoSDV RC | not measured yet |

Today a launch on a car takes three or four hand-typed arguments
(`car_profile:=`, `sensor_suite:=` or `lidar_model:=`, `use_gnss:=`) that
nothing ties together, and the per-car PWM files live inside our vehicle
package (`autosdv_vehicle_interface/params/cars/`), so another team can add a
car only by editing our repository.

## The rule: three layers, each Autoware's own

Autoware already separates these, and AutoSDV already passes the three
arguments through (`autosdv.launch.yaml`: `sensor_model`, `vehicle_model`;
`autosdv_autoware.launch.xml`: `vehicle_id`). The proposal is to use them for
what Autoware means by them, and to put nothing per-car anywhere else.

### 1. Model: what kind of sensor kit, what kind of vehicle (a package)

`sensor_model:=X` resolves to the packages `X_launch` and `X_description`;
`vehicle_model:=Y` to `Y_launch` and `Y_description`. A team with different
hardware authors its own packages, in its own repository (a submodule under
`src/` or an overlay workspace), and selects them by name. It never edits ours.

- `autosdv_sensor_kit` stays our sensor kit. Its suites (`robin_zed`,
  `vlp32c_zed`, ...) are hardware variants of the same kit, sharing one
  description; add `vlp16_zed` for car A.
- `autosdv_vehicle` stays our vehicle: geometry in `vehicle_info.param.yaml`,
  the vehicle interface in `autosdv_vehicle_launch`.

### 2. Instance: which physical car (`vehicle_id`, in `individual_params`)

Everything measured on one copy lives under
`individual_params/config/<vehicle_id>/`, which is exactly Autoware's
convention for sensor calibration:

```
individual_params/config/
  car_a/
    autosdv_sensor_kit/      sensor_kit_calibration.yaml, imu_corrector ...   (Autoware's layout)
    autosdv_vehicle/         actuator.param.yaml   <- PWM neutral/brake, steer centre/ends, ratio
    autosdv.preset.yaml      <- the deployment layer below
  car_b/ ...
  car_c/ ...
```

The `autosdv_vehicle/actuator.param.yaml` file replaces
`params/cars/<car>.param.yaml` and the `car_profile:=` argument: the vehicle
interface loads `config/<vehicle_id>/<vehicle_model>/actuator.param.yaml` last,
and the actuator refuses to start when it is missing (the rule `car_profile`
has today: the cars differ in their stop PWM).

`individual_params` is already our fork (`NEWSLabNTU/autoware_individual_params`,
branch `1.5.0`). A team keeps its fleet in its own `individual_params`
package, as Autoware intends: same package name, its own repository, its own
`vehicle_id`s. Their cars never appear in ours.

### 3. Deployment: how this car is launched (`autosdv.preset.yaml`)

One preset per instance, beside its calibration, in the format of the existing
perception and localization presets (a launch file of `arg` defaults):

```yaml
# individual_params/config/car_b/autosdv.preset.yaml
launch:
  - arg: {name: sensor_model,  default: autosdv_sensor_kit}
  - arg: {name: sensor_suite,  default: robin_zed}
  - arg: {name: vehicle_model, default: autosdv_vehicle}
  - arg: {name: use_gnss,      default: "false"}
```

```bash
just launch vehicle_id:=car_b                  # everything car B is
just launch vehicle_id:=car_b use_gnss:=true   # any preset value still overridable
just lab2 run car:=B                           # lab2's car key becomes vehicle_id:=car_b
```

`vehicle_id` gets no default and no environment variable (Autoware's
`$(env VEHICLE_ID default)` is dropped, matching the rule that the shell sets
nothing): on a car, a launch without it refuses, naming the instances that
exist. The simulator and the benches pass their own instance (`car_a`, the
car `actuator.yaml` and the plant model).

## What it costs

- Move `params/cars/car_{a,b,c}.param.yaml` from `autosdv_vehicle_launch` to
  `individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml`;
  `car_profile.load(name)` reads from there (by `vehicle_id`).
- `vehicle_interface.launch.xml`: `car_profile:=` becomes `vehicle_id:=`, which
  the launch already declares and Autoware's `vehicle.launch.xml` already passes
  through, so the parse-time requirement problem that kept `car_profile`
  optional goes away.
- `autosdv.launch.yaml`: include `config/$(var vehicle_id)/autosdv.preset.yaml`
  first; the suite and GNSS defaults come from it.
- `vlp16_zed` suite in `autosdv_sensor_kit_launch`.
- Rename every `car_profile:=` in launches, recipes, lab2 (`car:=B` maps to
  `vehicle_id:=car_b`), the tools' `--car-profile` (`--vehicle-id`), docs.
- Three submodules (individual_params, vehicle_launch, sensor_kit), pushed
  before the superproject pin.

## The lab2 stack's arguments, Autoware convention

`waypoint_follow.launch.xml` (the Lab 2 stack, `just lab2 vehicle` / `just lab2
stack VEHICLE_ID`) and the `autosdv.launch.yaml` it includes now sort their
vehicle and sensor arguments into Autoware's layers. Both declare `vehicle_id`
first and include `individual_params/config/<vehicle_id>/autosdv.preset.yaml`
before the arguments it defaults, so a preset value beats the file's default
and the command line beats both.

| argument | before | after | layer |
|---|---|---|---|
| `sensor_model` | file default `autosdv_sensor_kit`; waypoint_follow: hardware:=sim only | preset default (file default as fallback), passed to `autosdv.launch.yaml` too | model |
| `vehicle_model` | file default `autosdv_vehicle` | preset default (file default as fallback) | model |
| `vehicle_id` | `$(env VEHICLE_ID default)`: Autoware's `default` calibration on every car | **required**, no environment; sims pin and benches default to `car_a` | instance |
| `config_dir` (URDF calibration) | `individual_params/config/default/<sensor_model>` | `individual_params/config/<vehicle_id>/<sensor_model>` | instance |
| actuator PWM file | `car_profile:=` (optional at parse, refused at start) -> `autosdv_vehicle_interface/params/cars/<car>.param.yaml` | `individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml`, from `vehicle_id` | instance |
| `sensor_suite` | file default (`robin_zed` in waypoint_follow, loc-check, survey-align; `vlp32c_zed_imu` in autosdv.launch.yaml) | preset default (car A `vlp16_zed`, B and C `robin_zed`); loc-check and survey-align take the preset's | preset |
| `use_gnss` | autosdv.launch.yaml: empty (the suite's) | preset default; waypoint_follow still passes `false` (indoor hall, ArUco) | preset |
| `camera_model zedxm`, `imu_source zed`, `use_ntrip false`, the `launch_*` switches | hardcoded at the include | unchanged: what the lab needs, not what the car is | per-launch (lab) |
| `route` / `course` / `course_dir`, `laps` | per launch | unchanged | per-launch |
| `localization` (aruco, ground_truth), `hardware` (car, sim), `is_simulation` | per launch | unchanged | per-launch |
| `tracker_class`, `tracker_param_file`, `speed_controller_*`, `actuator_param_overrides` (the lab's measured ratio) | per launch | unchanged | per-launch |
| `rviz`, `viewer_display`, `use_sim_time`, the bench switches | per launch | unchanged; `just lab2 stack` hands RViz `viewer_display` (the ZED runs without DISPLAY) | per-launch |

Commands:

```bash
just lab2 stack car_b                            # car B's preset, hall_demo, reference laws
just lab2 stack car_a sensor_suite:=vlp16_zed    # a preset value on the command line
just lab2 vehicle                                # the graded stack: car:=B in ~/lab2/lab2.yaml -> vehicle_id:=car_b
```

## Not part of this

- Measuring car C, or any per-car sensor extrinsics: each copy's
  `sensor_kit_calibration.yaml` starts as a copy of the kit default and must be
  measured, not trusted.
- Splitting `autosdv_sensor_kit` into one package per suite: only worth it if
  the suites' descriptions (URDF, frames) diverge.
