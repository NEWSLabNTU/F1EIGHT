# RC car simulator: Autoware on a laptop, up to 40 km/h

Status: design, for review (2026-10-17). Nothing here is implemented yet.

## Why

Lab 2 Part 1 becomes simulation only (decided 2026-10-16): students write a
speed controller and prove it in simulation, with no recording from a car. The
car can reach about 40 km/h (11 m/s), which is never driven in the hall for
safety, but a controller should face it. The simulation must run the Autoware
stack on a student laptop without a GPU.

The simulators we have are not fit for that speed:

- Autoware's `simple_planning_simulator` (1.5.0) offers only kinematic models
  with first-order lag and dead time (`IDEAL_STEER_*`, `DELAY_STEER_*`,
  `ACTUATION_CMD*`, `LEARNED_STEER_VEL`). None has tire slip; its README lists
  slip under future extensions. It has no plugin mechanism, and its inputs are
  a target speed or acceleration, which would bypass a PWM-level controller.
- Our lab bench plants (`motor_plant.py`, the coach lab's `bench_plant.py` and
  `vehicle_bench_plant_node.py`, the waypoint lab's `kinematic_bench.py`) are
  kinematic bicycles with a first-order speed response, valid at walking pace.

At 11 m/s a kinematic model is wrong by a factor of two or more: a steady turn
asks for a lateral acceleration v²·tan δ / L, about 19 m/s² at 11 m/s and
0.05 rad, while the tires can give μg ≈ 6–10 m/s². Above 2–3 m/s the tires,
not the geometry, decide the motion.

## Decisions (2026-10-17)

| Question | Decision |
|---|---|
| Approach | Upgrade our own Python plant behind the existing interface (PWM in, sensors out), replacing `simple_planning_simulator` |
| Student task | Tune the car's own speed controller (`AckermannSpeedPID` in the vehicle interface) and its related coefficients; Autoware plans and steers |
| Student track | One straight, so students concentrate on speed and braking control. Its length follows from the graded speed profile (below) |
| Steering limit | The physical lock, `max_steer_angle` 0.349 rad from `vehicle_info.param.yaml`. The 0.05 rad servo saturation (`docs/known-config-defects.md` §8) is a configuration defect to be measured on the car, not a property to simulate |
| Where | AutoSDV, a new package `src/sim/autosdv_rc_sim` |
| Duality | The real vehicle interface runs in simulation unchanged; only its two hardware touchpoints get simulated backends (`hardware:=real\|sim`) |
| Grading | Deterministic and independent of CPU speed: a lockstep simulation clock in one process, running the same logic classes as the ROS nodes |
| Seeing the run | Grading is headless, but the graded run is shown in Autoware's RViz: the trace is replayed with the map and the vehicle model |
| Drivetrain | Rear-wheel drive |
| Track sizing | From the speeds to be held, the hold time and the planner's limits, not drawn; curves remain possible for demos, not graded |

Alternatives considered: forking `simple_planning_simulator` with a C++ slip
model (students would have to publish Autoware's actuation command instead of
PWM, and the fork must follow upstream), and Project Chrono's RC car with
Chrono::ROS (most faithful, about a core, a heavy install, hard for students to
debug; kept as a possible cross-check of this model).

## Architecture

### The simulation as a dual of the car

The vehicle interface touches hardware in exactly two places, and the simulator
replaces those and nothing else:

- `actuator.py` (and `safety.py`) write PWM through an `Adafruit_PCA9685`
  object, using only `set_pwm_freq()` and `set_pwm(channel, 0, value)`
  (channel 0 motor, channel 1 steering).
- `velocity_report.py` uses `Jetson.GPIO` (`setmode`, `setup`,
  `add_event_detect(pin, RISING, callback)`) and times the hall-sensor edges
  itself.

```
car:        actuator ─set_pwm→ PCA9685 (I2C) → ESC/servo → chassis → magnet → GPIO edge → velocity_report
simulation: actuator ─set_pwm→ SimPCA9685 ───→ rc_sim plant (physics) → hall edge → SimGPIO → velocity_report
```

A `hardware:=real|sim` parameter (beside today's `dry_run`) selects the backend.
`SimPCA9685` exposes the same two methods and hands the raw counts per channel to
the plant; `SimGPIO` exposes the same three calls and delivers each hall edge to
the same callback, with the edge's true time (an optional argument the real path
does not use, so message latency does not become speed noise). The plant tracks
the wheel angle and emits an edge whenever one of the 12 markers passes, so the
real edge-timing arithmetic, its filter and its timeout produce the speed. The
`Jetson.GPIO` import moves behind the backend selection, so the package imports on
a laptop. The IMU comes from the plant on the topic the ZED wrapper uses, and the
real `imu_corrector` runs downstream.

Everything else in the vehicle interface (the steering status, the gear and
control-mode managers, the safety shell, the speed-controller plug-in) has no
hardware and runs as it is. The student's `SpeedPID` is the class the car's
actuator loads, inside the same code around it. The lab bench plants that read
the actuator's debug PWM topic, and their copy of the hall model, retire.

### Two modes, one set of logic classes

**Headless grading (lockstep, deterministic).** One Python process steps the
whole loop in a fixed order on a simulated clock (1 ms physics, 10 ms control):

    plant → hall edges → speed estimate → safety shell + SpeedPID → path tracker → plant

No ROS messages and no wall clock are involved, so a submission gives
bit-identical results on any machine, and runs as fast as the CPU allows. For
this to stay the car's code, the vehicle interface's logic lives in plain classes
that the ROS nodes only wrap: `longitudinal.py` already does; the edge-timing
speed estimate moves out of `velocity_report.py` into a `VelocityEstimator` class
used by both. A test feeds the node and the harness the same edges and requires
the same output.

**Full stack, interactive (ROS, simulation clock).** Autoware in
planning-simulator mode, the real vehicle-interface nodes with `hardware:=sim`,
and the plant, which publishes `/clock`; every node runs with `use_sim_time`. It
runs at real time or slower, for watching and debugging in RViz. Results from
this mode are not graded: separate processes on a simulated clock still race
within a tick (which of the speed report and the actuator acts first depends on
CPU and delivery), so they are reproducible only approximately.

```
Autoware, planning-simulator mode
  map (lanelet2), planning, control (trajectory follower, vehicle_cmd_gate),
  system / MRM, ADAPI, RViz            -- no sensing, perception, NDT, CUDA
        │ /control/command/control_cmd  (target speed, steering angle)
        ▼
autosdv_vehicle_interface, hardware:=sim  -- unchanged nodes
  actuator (safety shell, the student's SpeedPID) ─set_pwm→ SimPCA9685
  velocity_report ←edges─ SimGPIO
        ▲                         │ PWM counts
        │ hall edges              ▼
autosdv_rc_sim plant  (1 kHz physics, publishes /clock)
        │ /sensing/imu/imu_data, /localization/kinematic_state,
        │ /localization/acceleration, TF map → base_link (true pose)
        ▼  back to Autoware
```

One command starts each mode: `just sim rc [map:=…] [submission:=DIR]` and
`just sim grade DIR`.

**Seeing a graded run.** `just sim grade DIR --view` grades headlessly, then
replays the run in Autoware's RViz at real time: the straight's lanelet map, the
vehicle model moving along it, the target and measured speed, the PWM, the
wheel slip, and every violation marked. The replay publishes the recorded trace
(pose, twist, the vehicle status topics) on the topics Autoware's RViz
configuration reads, so what the student sees is exactly what was graded. It
needs no planner and stays cheap on a laptop. The interactive mode remains for
running Autoware live.

**Proof of determinism.** A test grades one submission twice, once with the CPU
loaded by a stress job, and requires identical marks and identical traces.

## The vehicle model

All parameters live in one YAML, each tagged `measured`, `config` (taken from
AutoSDV's own files), `literature` or `guess`, so the file says how far to
trust each number and what to measure next.

### Electronic speed controller

- PWM counts in (`min_pwm` 360, `init_pwm` 370, `max_pwm` 470, `brake_pwm` 340
  from `actuator.yaml`); one count is about 1 % of the throttle span, and the
  quantization is kept.
- Neutral deadband (about ±5 counts), throttle ramp (a rate limit on duty),
  drag brake at neutral, brake below neutral, and reverse only after a stop and
  a return to neutral.
- Duty d ∈ [−1, 1] out; battery sag scales the applied voltage with the motor
  current (a series resistance).

### Motor and drivetrain

A DC motor through a fixed gear ratio G drives the wheels:

    τ_wheel = G · (K_t / R) · (d · V_bat − K_e · G · ω_wheel)

The back-EMF term gives a natural top speed (about 11 m/s at full duty with the
default parameters); the electrical time constant (under 1 ms) is dropped.
The AutoSDV chassis is rear-wheel drive: the motor drives the rear axle only; the front wheels roll freely.

### Wheels and friction

Front and rear wheel speeds are states, so the driven wheels can spin under hard
throttle and lock under hard braking. Static friction uses a Karnopp stick band:
while |ω| is below a small threshold and the drive torque is below the
breakaway torque, the wheel is held; above it, Coulomb plus viscous friction
apply. This gives the start-up behaviour a deadband-only model misses: a command
that holds speed cannot start the car from rest.

### Tires and chassis

The single-track drift model (CommonRoad "STD"): states x, y, ψ, v, β, ψ̇,
ω_f, ω_r, δ. Pacejka combined slip at each axle, longitudinal slip from the
wheel speeds, lateral slip from the slip angles, and axle load transfer from
the longitudinal acceleration with the centre-of-mass height. Below about
0.3 m/s it blends smoothly into the kinematic bicycle (CommonRoad's tanh blend,
with a minimum speed in the slip denominator) so it is stable at standstill.

Resistances: rolling resistance C_r0 and aerodynamic drag C_d·v².

### Steering servo

Dead time and a rate limit, saturating at the physical lock.

### Sensors

The hall-sensor model the bench already has (12 markers per revolution, edge
timing, the shell's filter, a timeout, decaying to zero once edges stop), the
IMU with bias and noise, and the steering status (which, as on the car,
republishes the command: the car has no steering feedback).

### Starting parameters

| Parameter | Value | Source |
|---|---|---|
| Wheelbase, track | 0.319 m, 0.263 m | config (`vehicle_info.param.yaml`) |
| Steering lock | 0.349 rad | config |
| PWM range | 360 / 370 / 470, brake 340 | config (`actuator.yaml`) |
| Mass | 3.5 kg | guess: to be weighed |
| Yaw inertia I_z | 0.045 kg·m² | literature (F1TENTH Traxxas: 0.0471) |
| Front/rear axle distance | 0.15 / 0.17 m | literature, to be measured (balance point) |
| CoM height | 0.07 m | literature |
| Cornering stiffness per axle | 90–100 N/rad | literature (F1TENTH: about 94 / 101) |
| Friction coefficient μ | 0.8 | guess: depends on the floor; tilt or skidpad test |
| Pacejka B, C, E | to fit μ and the stiffness | literature shape |
| Wheel radius | 0.0525 m | config (`velocity_report.yaml` 10.5 cm diameter; `vehicle_info` 0.052) |
| Battery | 3S LiPo, 11.1 V nominal, 0.03 Ω internal | guess |
| Motor | brushless, K_v 3000 rpm/V, so K_t = K_e = 60/(2π·K_v) = 0.00318 N·m/A, winding + ESC resistance 0.03 Ω | **mock**: typical 1/10-scale motor, not measured |
| Total gear ratio G (pinion/spur × differential) | 14.5 : 1 | **mock**: chosen so the no-load top speed is about 12.5 m/s and the loaded top speed about 11 m/s |
| Rolling resistance, drag | from a coast-down | guess |

Sources: F1TENTH gym defaults (`f110_env.py`), CommonRoad vehicle models
(`vehicle_dynamics_std.py`), Liniger et al., "Optimization-based autonomous
racing of 1:43 scale RC cars" (drivetrain form), Karnopp (1985) for stiction.

## The track

Students drive one straight. The track is a single lanelet whose length follows
from the speed profile that is graded, so the car can accelerate to each speed,
hold it long enough to measure, and brake to a stop on a stop line:

    L = Σ_i (v_i² − v_{i−1}²) / (2a)  +  Σ_i v_i · T  +  v_top² / (2d)  +  L_runout

With the planner's limits a = 2.5 m/s² and d = 2.5 m/s², holds of T = 3 s and
the graded profile 0 → 2 → 6 → 11 m/s → stop:

| Segment | Length | Time |
|---|---|---|
| accelerate 0 → 2 m/s | 0.8 m | 0.8 s |
| hold 2 m/s | 6.0 m | 3.0 s |
| accelerate 2 → 6 m/s | 6.4 m | 1.6 s |
| hold 6 m/s | 18.0 m | 3.0 s |
| accelerate 6 → 11 m/s | 17.0 m | 2.0 s |
| hold 11 m/s (40 km/h) | 33.0 m | 3.0 s |
| brake 11 m/s → stop on the stop line | 24.2 m | 4.4 s |
| run-out beyond the stop line | 10 m | — |
| **total** | **about 115 m** | **about 18 s** |

**Changed 2026-10-18:** the ESC's regeneration was lowered to a light drag brake
(`esc_regen_current_limit` 8.65 → 2.5 A: 1.5-2.0 m/s² with road load, against 3.0-3.5
for the ESC brake), so a controller needs a braking mode to stop at 2.5 m/s²
(docs/design/lab2-part1-sim-design.md). `AckermannSpeedPID` has none, so the committed
default track brakes at 1.5 m/s²: 40.3 m and 7.3 s for the last row, 131.5 m and 20.7 s
in all.

The generator (`just sim make-track --speeds 2,6,11 --hold 3 --accel 2.5
--decel 1.5`) prints this table, writes the lanelet2 map (one straight lane,
the start pose, the stop line) and the speed-limit profile, and refuses a
profile whose segments cannot reach their speed. Changing a speed or the hold
time resizes the straight. Curves (a radius list) remain possible for
demonstrations; a curve of radius R is driven at √(a_lat·R), with a_lat about
3 m/s². They are not graded.

## What students do (Part 1)

Students work on the speed controller the car runs: `AckermannSpeedPID` in
`autosdv_vehicle_interface/speed_controller.py`, loaded by the actuator. They
study how the textbook PID becomes that code (sampled integral, derivative on
the measurement with a low-pass filter, output clamp, integral limit,
conditional integration) and tune its coefficients in the simulator. Nothing is
written from scratch; the deliverable is a parameter file and a report.

The coefficients they may set, as `speed_controller_params` (the keys the class
and the actuator already read):

| Key | Meaning | Production value |
|---|---|---|
| `kp_speed` | proportional gain [PWM counts per m/s] | 12.0 |
| `ki_speed` | integral gain | 10.0 |
| `kd_speed` | derivative gain, on the measured speed | 0.001 |
| `integral_limit` | the integral's clamp [counts] | 50.0 |
| `enable_conditional_integration` | freeze the integral while the output saturates | true |
| `derivative_filter_alpha` | the derivative's low-pass filter | 0.15 |
| `pwm_output_alpha` | the actuator's output filter | 0.25 |
| `max_pwm_offset` | the controller's output limit around `init_pwm` [counts] (new) | 55, today's `(max_pwm − min_pwm) // 2`; at most `max_pwm − init_pwm` = 100 |

Anything outside this list is refused, as the Lab 2 tracker file is today. The
simulation replaces the car recordings and the per-car models: no car CSV, no
per-car calibration.

Graded scenarios, all on the straight, in the headless lockstep simulator:

- **Speed holds** at 2, 6 and 11 m/s (the profile above): speed error while
  steady, overshoot after each step, time to reach each new speed.
- **Launch from rest:** static friction must be overcome without a lurch; the
  rear wheels' slip ratio stays under a limit (no wheel spin).
- **Braking to the stop line** from 11 m/s: stop error, and no wheel lock (slip
  ratio under a limit while braking).
- **An emergency stop** at 11 m/s (the target drops to 0 at once): the safety
  shell brakes; the controller must not fight it, and must resume cleanly.

Model variants (mass, μ, battery voltage, motor constants, ESC deadband) replace
the per-car variants, so coefficients tuned to one exact model lose marks.
Autoware is not needed to grade: the straight's speed profile is replayed as the
target, which is what Autoware's planner would command on it.

**A finding to settle first: the production controller cannot command full
throttle.** `AckermannSpeedPID` limits its output to `init_pwm ±
(max_pwm − min_pwm) // 2`, that is 370 ± 55, so at most PWM 425 against the ESC's
470. If full duty gives about 11 m/s, the controller tops out near half of that,
well short of 40 km/h. Decided: the output limit becomes a coefficient,
`max_pwm_offset`, read by `AckermannSpeedPID` with today's value as its default
(so the car's behaviour is unchanged until someone sets it), and bounded by the
actuator's own clamp (at most `max_pwm − init_pwm`). This is a change to the
vehicle interface (a submodule), made in its tracking branch first.

## Running on a laptop

Planning-simulator mode measured on this workstation: about 2.5 GiB peak and 2
cores steady, plus RViz about 1.4 GiB and 1 core
(`docs/reports/host-resource-measurements.md`). The plant adds under 0.2 core.
Target: a 4-core, 8 GB laptop with Ubuntu 22.04 and Autoware 1.5.0 from the
Debian packages, no NVIDIA driver; TensorRT never loads because perception is
off. The headless grading mode needs only Python.

## Validation

- Unit tests of the physics: the steady top speed matches the motor parameters;
  a coast-down matches drag and rolling resistance; steady cornering keeps
  lateral acceleration at or below μg; full throttle from rest spins the driven
  wheels; neutral does not creep; braking locks the wheels only above the
  friction limit.
- A cross-check against the F1TENTH single-track model at moderate speed.
- Calibration hooks: scripts to refit μ, the motor and the resistances from car
  data (`just coach fit-motor`, a coast-down, a skidpad) once the car can drive.
- The laptop budget, measured: under 3 cores and 4 GB without RViz.

## Open questions

1. Decided (2026-10-17): no separate feed-forward; students tune the
   production controller; its output limit becomes the coefficient
   `max_pwm_offset`.
2. The motor and gear ratio are mocked (K_v 3000 rpm/V, G 14.5:1, 3S) until
   someone reads them off the car; a full-throttle stall then gives about 105 N
   at the wheels (the ESC limits the current to 120 A), far above the rear tires' grip (about 15–20 N), so full
   throttle from rest spins the rear wheels, as RC cars do. Rear-wheel drive is
   decided.
