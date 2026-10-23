# planning_profile:=rc

Autoware's planning files, changed where the RC car (and the classroom it drives in)
needs them. Loaded by `launch/components/autosdv_planning_component.launch.xml` when
`planning_profile:=rc`; the default `stock` loads `autoware_launch`'s files unchanged.
Each file here is a copy of upstream's (Autoware 1.5.0) with the changed lines marked
`AutoSDV rc:`. Re-copy after an Autoware upgrade.

| file | change | why |
|---|---|---|
| `start_planner.param.yaml` | `enable_back` false, freespace pull-out off | the car cannot reverse; a pull-out that backs up first leaves it parked at the start (Lab 3's second lap, from a stop 13 deg off the lane) |
| `goal_planner.param.yaml` | backward-arc and freespace parking off | both reverse; freespace parking also runs its own planning thread, and the behavior planner once crashed tearing the goal planner's threads down on a route change |
| `behavior_path_planner.param.yaml` | path interval 2.0 -> 0.5 m | a 2 m interval cuts a 1.5 m corner into straight chords |
| (module preset) | `launch_start_planner_module` false | from a stop a few degrees off the lane its pull-outs reverse, or with reversing off find no forward path and hold the car; a loop has nothing to pull out of. Set in the component, not a file |
| `trajectory_checker.param.yaml` | curvature 1.0 -> 1.2 | the car turns at 0.88 m radius (curvature 1.14); 1.0 rejects curves it can drive |

Used by Lab 3 (`lab3_run.launch.xml`, `lab3_sim_lap`). `autosdv_launch/config/planning/`
is an older, unused copy of upstream's tree: nothing loads it.
