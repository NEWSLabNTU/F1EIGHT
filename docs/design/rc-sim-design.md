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

Alternatives considered: forking `simple_planning_simulator` with a C++ slip
model (students would have to publish Autoware's actuation command instead of
PWM, and the fork must follow upstream), and Project Chrono's RC car with
Chrono::ROS (most faithful, about a core, a heavy install, hard for students to
debug; kept as a possible cross-check of this model).

## Architecture

```
Autoware, planning-simulator mode
  map (lanelet2), planning, control (trajectory follower, vehicle_cmd_gate),
  system / MRM, ADAPI, RViz            -- no sensing, perception, NDT, CUDA
        │ /control/command/control_cmd  (target speed, steering angle)
        ▼
autosdv_vehicle_interface actuator      -- unchanged: safety shell,
  speed controller plug-in = the student's SpeedPID, dry run (no PCA9685)
        │ /autosdv/actuator_node/debug/pwm_values  (motor, steering PWM)
        ▼
autosdv_rc_sim plant  (new)             -- 1 kHz physics, 5 substeps per 200 Hz tick
        │ /vehicle/status/velocity_status   (hall-sensor speed)
        │ /vehicle/status/steering_status
        │ /sensing/imu/imu_data             (IMU with bias and noise)
        │ /localization/kinematic_state, /localization/acceleration
        │ TF map → base_link                (true pose, optional noise)
        ▼  back to Autoware and the actuator
```

The plant takes exactly the PWM the actuator would write to the PCA9685, so the
student's class, the actuator's modes (emergency brake, full stop, coast,
deadband hold, active control), the watchdogs and the fault handling are all in
the loop, as on the car. Localization is the simulator's true pose; position
noise and dropouts are options for tests, not defaults.

One command starts it: `just sim rc [map:=…] [submission:=DIR]`.

A headless mode runs the plant and the actuator logic without Autoware, faster
than real time and deterministic (fixed seed, fixed step), for grading and for
`selfcheck.py`. It replays a speed profile (or Autoware's planned profile
recorded once per track) instead of the planner.

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

A generated lanelet2 loop of about 150–200 m, produced like `make-uturn` by a
generator (`just sim make-track`), with two long straights where 11 m/s is
reachable, curves of about 20 m, 8 m and 4 m radius, and one hairpin. The
generator writes the lanelet2 map, a start pose and a speed-limit profile. The
Lab 2 U-turn loads too, for the low-speed case.

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
2. Grading: headless only (fast, deterministic), or also a run with Autoware in
   the loop?
3. Rear-wheel or all-wheel drive on the AutoSDV chassis; the motor and gear
   ratio; the battery voltage (the drive battery is 3S, 11.1 V nominal).
4. Track size: does a 150–200 m loop fit the planner's defaults (lane width,
   curvature limits) at small-car scale?
