# Lab 2: each team builds its own Autoware vehicle model

Status: proposal, 2026-10-20. Decided with the TA: the team selection merges into
`vehicle_model:=`, the template is a copy of `autosdv_vehicle_launch`, a team has
full control of its vehicle interface, and the car-specific files stay keyed to
the physical car. Not implemented.

## What Lab 2 becomes

"Build your own vehicle interface package for Autoware." Autoware already chooses
a vehicle by name: `vehicle_model:=X` resolves

| package | provides |
|---|---|
| `X_description` | `config/vehicle_info.param.yaml` (wheelbase, steering limit; the planner, controller and interface all size against it), the URDF |
| `X_launch` | `launch/vehicle_interface.launch.xml` |

and `vehicle_id:=` separately picks the physical car's calibration in
`individual_params`. Our repository `autosdv_vehicle_launch` is already that shape,
with a third package for the nodes:

| ours (the reference) | a team's |
|---|---|
| `autosdv_vehicle_description` | `team3_vehicle_description` |
| `autosdv_vehicle_interface` | `team3_vehicle_interface` |
| `autosdv_vehicle_launch` | `team3_vehicle_launch` |

So a team is `vehicle_model:=team3_vehicle`, and the reference is
`vehicle_model:=autosdv_vehicle`:

```bash
just lab2 run vehicle_model:=team3_vehicle vehicle_id:=car_c route:=lab2_uturn
just lab2 run vehicle_id:=car_c route:=lab2_straight_2m          # the reference model
```

`speed_controller_class:=`, `team_dir:=`, `submission:=` and the `team:=0`
workaround leave the Lab 2 command lines: the class a model uses lives inside its
own package.

## What the team owns, and what it does not

**The team owns its whole vehicle interface**: the template is a copy of
`autosdv_vehicle_launch`, renamed, so a team has its own actuator node, speed
controller, velocity report and launch file, and may change any of it. Lab 2 is
about control: the contract it teaches is Autoware's vehicle interface
(`/control/command/control_cmd` in; `/vehicle/status/velocity_status`,
`steering_status`, `gear_status`, `control_mode` out).

**The TA owns everything else**: the paths (`courses/`), the path tracker
(reference `PathTracker` and its parameters), localization (the ArUco stack, being
made standalone: `just aruco-map`), `vehicle_cmd_gate`, the operation-mode manager,
`lab2_run`'s disengage, and the cars' `individual_params`.

## The car's files stay keyed to the car

A car's PWM points (neutral, brake, ESC start/hold, servo centre and ends) are a
property of its ESC and servo, not of a team. They stay in
`individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml`, a fixed
path whatever `vehicle_model` is, and the template's launch loads that file last,
exactly as `autosdv_vehicle_launch` does today. A team's package must not carry a
copy: the cars differ in their stop PWM (A 380, B 395, C 385), and a wrong one
drives a car instead of stopping it.

The same holds for `vehicle_info.param.yaml`: the template's
`team3_vehicle_description` installs autosdv_vehicle_description's file (by
reference, not a copy), because the wheelbase and steering limit are the car's.
A copy drifted once already (0.5 rad and 0.340 m against 0.349 and 0.319).

A team's speed tuning is the team's: `team3_vehicle_launch/config/` holds it, and
`individual_params/config/<vehicle_id>/autosdv_vehicle/speed_controller.param.yaml`
stays the reference model's tuning for that car.

## Safety, with the team in full control

A team can change or break its own stop logic, so the guarantees come from
outside its package:

1. **The transmitter selector switch** (`manual-override-and-kill-switch.md`):
   the TA takes the car in hardware whatever the team's code does. Every powered
   team run has a TA holding the transmitter; `lab2 run` already asks before it
   engages.
2. **A bench gate before any car**, `just lab2 gate vehicle_model:=team3_vehicle`:
   the team's model in the RC simulator (`hardware:=sim`) must
   - stop at the car's neutral PWM (the `individual_params` value, read back from
     what the model actually commands),
   - stop when `control_cmd` stops arriving (watchdog),
   - never arm the ESC's reverse (no command at or below `reverse_arm_pwm`
     unless a reverse is asked for),
   - stop on an emergency / MRM,
   - and drive the route (today's gate checks).
   A model that fails does not go on a car.
3. **Autoware outside the package** (`vehicle_cmd_gate`, the operation-mode
   manager, `lab2_run`'s disengage on localization loss or timeout) still acts,
   but only by commanding stops through the team's interface; 1 and 2 are what
   protect the car.

## Workflow

- `just lab2 new-team 3` generates `team3_vehicle_launch/` (the three packages)
  from `autosdv_vehicle_launch`, renamed.
- Teams submit the repository; the TA puts every team under one overlay
  workspace, `~/lab2/ws/src/`, and builds it once. Distinct package and Python
  module names let all teams build side by side; a broken package fails only its
  own build.
- Part 1 (the RC simulator) loads the same model, `vehicle_model:=team3_vehicle
  vehicle_id:=sim`, so one submission serves both parts.
- Bags record `vehicle_model` and the package's version (git hash); `grade` and
  `audit` read it from there.

## Before the release: the reference model first

Everything above is first made true for `vehicle_model:=autosdv_vehicle` on our
own cars, and only then is the template cut from it:

1. every Lab 2 entry point (run, vehicle, bench, gate, sim, grade, audit) selects
   by `vehicle_model`, and the reference runs with no controller arguments;
2. the car C straight-line and U-turn runs on the reference model;
3. the gate's safety checks above, passing on the reference model and failing on
   deliberately broken copies (wrong neutral, no watchdog, reverse arming);
4. `new-team`, then one generated team package through gate, car and grade.

## Open

- Where the template comes from at release: a copy committed per release, or
  generated from the `autosdv_vehicle_launch` the release pins.
- How Part 1's grader finds the controller inside a team's interface, now that
  the interface, not a class, is the submission.
