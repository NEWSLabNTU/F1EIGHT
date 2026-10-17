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
| Student task | The speed controller only (`SpeedPID`: target speed → PWM). Autoware plans and steers |
| Map | A generated lanelet2 track sized for 40 km/h; the Lab 2 U-turn also loads |
| Steering limit | The physical lock, `max_steer_angle` 0.349 rad from `vehicle_info.param.yaml`. The 0.05 rad servo saturation (`docs/known-config-defects.md` §8) is a configuration defect to be measured on the car, not a property to simulate |
| Where | AutoSDV, a new package `src/sim/autosdv_rc_sim` |
| Duality | The real vehicle interface runs in simulation unchanged; only its two hardware touchpoints get simulated backends (`hardware:=real\|sim`) |
| Grading | Deterministic and independent of CPU speed: a lockstep simulation clock in one process, running the same logic classes as the ROS nodes |
| Track | Sized from speeds and hold times, not drawn |

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
Rear-wheel drive or all-wheel drive is a parameter (to be checked on the car).

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
| Motor K_t, K_e, R, gear G | to give 11 m/s at full duty | guess: fit from speed steps |
| Rolling resistance, drag | from a coast-down | guess |

Sources: F1TENTH gym defaults (`f110_env.py`), CommonRoad vehicle models
(`vehicle_dynamics_std.py`), Liniger et al., "Optimization-based autonomous
racing of 1:43 scale RC cars" (drivetrain form), Karnopp (1985) for stiction.

## The track

The track is sized from the speeds to be tested and how long each must be held,
not drawn. A straight must let the car accelerate to v, hold it for T seconds
(the steady window that is measured) and brake:

    L_straight = v² / (2a) + v·T + v² / (2d)

and a curve of radius R is driven at v_curve = √(a_lat · R). With the planner's
limits a = 2.5 m/s², d = 3.5 m/s² and a_lat = 3 m/s² (well under μg ≈ 7.8):

| Top speed of the straight | Hold T | Length |
|---|---|---|
| 11 m/s (40 km/h) | 3 s | 24 + 33 + 17 = 75 m |
| 11 m/s | 5 s | 97 m |
| 6 m/s | 3 s | 7 + 18 + 5 = 31 m |

| Curve radius | Curve speed |
|---|---|
| 20 m | 7.7 m/s |
| 8 m | 4.9 m/s |
| 4 m | 3.5 m/s |

Default loop (holds of 3 s): a 75 m straight at 11 m/s, a 180° turn of 20 m
radius (63 m), a 31 m straight at 6 m/s, an 8 m curve, a 4 m hairpin, and a
return straight closing the loop; about 380 m, a lap of about 60 s. The
generator (`just sim make-track`) takes v, T, a, d, a_lat and the radii, prints
every segment's length and the lap, refuses a segment too short to reach its
speed, and writes the lanelet2 map, the start pose and the speed-limit profile.
The Lab 2 U-turn also loads, for the low-speed case.

## What students do (Part 1)

They implement `SpeedPID` (target speed → PWM), from the textbook PID to code,
as now. The simulation replaces the car recordings and per-car models:

- No car CSV and no per-car calibration. If the feed-forward stays, students fit
  it from a speed-step run they make in the simulator (`just sim speed-steps`).
- Grading runs closed-loop scenarios in the headless simulator: speed holds at
  about 2, 6 and 11 m/s, a launch from rest (static friction, wheel spin), hard
  braking before a curve, a lap following Autoware's speed profile (speed error
  on the straights and in the curves), and safety checks (slip ratio, lane
  excursion). The existing contract tests (no ROS imports, reads `init_pwm`,
  reset, finite output) stay.
- Model variants (mass, μ, battery, motor) replace the per-car variants, so a
  controller tuned to one exact model loses marks.

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

1. Keep the feed-forward (students fit it from a simulator speed-step run) or a
   plain PID?
2. Grading is headless and deterministic (decided); a full-stack run is for
   debugging only. Should a full-stack lap still be required as a smoke test?
3. Rear-wheel or all-wheel drive on the AutoSDV chassis; the motor and gear
   ratio; the battery voltage (the drive battery is 3S, 11.1 V nominal).
4. Track size: does a 150–200 m loop fit the planner's defaults (lane width,
   curvature limits) at small-car scale?
