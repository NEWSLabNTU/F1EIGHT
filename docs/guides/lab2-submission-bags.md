# Lab 2 submission bags: record, check, grade

Instructor-facing. Code: `autosdv_waypoint_lab` (`record.py`, `bag_meta.py`,
`check_bags.py`, `route_grade.py`, `grading.py`).

## Record

```bash
just lab2 vehicle route:=lab2_uturn ...          # the stack first
just lab2 record --car A --team 3 --name uturn_1  # then engage; Ctrl-C ends
```

Writes `rosbags/lab2_<name>`. The course is read from the running path
publisher, not from an argument. Topics: `bag_meta.RECORD_TOPICS` (localized
state, marker fixes, velocity status, tracker and gate commands, the actuator's
`pwm_values`/`pid_values`/`control_values`, progress, MRM, operation mode,
diagnostics graph, IMU, `/tf`, `/tf_static`), plus:

- `/lab2/camera/image/compressed`: the ZED wrapper's own
  `.../rgb/color/rect/image/compressed` (image_transport), decoded, resized to
  960x600 and re-encoded at 10 Hz by `lab2_record`.
- `/lab2/record_meta`: latched JSON (also `lab2_meta.json` beside the bag):
  course name, `route_kind`, `route_hash`, `course_hash`, car, team, git commit.

`route_hash` covers `path.csv`, laps, `v_ref`, start pose and the path file
name, not the markers, so a surveyed copy of a template hashes like the template
and a doctored path, speed or lap count does not.

## Check

`just lab2 check-bags BAG... [--course DIR ...] [--car A] [--team 3]` rejects a
bag for: a missing required topic, no metadata, camera under 8 Hz / not
960x600 / not covering the run, route hash differing from the template (or the
surveyed copies given with `--course`), `v_ref` or commanded speed not
0.8 m/s, a route other than `lab2_uturn` (bags of the old straight and circles
are rejected), other than 1 lap, the localized pose more than 0.3 m from the
path, MRM, disengage, never engaged, unfinished run. With several bags: no route
twice, one car, one team. Exit 1 on any rejection.

## Grade

`just lab2 grade --score BAG [--reference REFBAG]`. Definitions, the points
curve and every constant are in the `route_grade.py` docstring and header.
Summary: steady window = first 90 % of 0.8 m/s to the start of the stop ramp;
each window sample is put on the straight legs or in the U-turn by projecting
the localized pose onto the path (the turn's arc-length interval is read from
`path.csv`, so a regenerated radius needs no grader change); full marks at
<= 1.5x the car's reference value, linear to zero at 4x (reference floored at
0.02 m/s, 0.05 m); fixed absolute lines without a reference.

| metric (`grade` prints) | points |
|---|---|
| `speed_rmse_straight`: RMS of v − 0.8 on the legs | 15 |
| `speed_rmse_turn`: RMS of v − 0.8 in the U-turn | 10 |
| `overshoot`: peak above 0.8 before the stop ramp | 5 |
| `stop_error`: \|d\| + max(0, d) from the stop line, d > 0 past it | 15 |

`speed_rmse` (whole window) is printed but not scored. A bag is **INVALID** and
scores 0, with the reasons printed, unless it was engaged, finished at the
stop, raised no MRM, never disengaged, and kept the localized pose within
0.3 m of the path from engage to rest (`path_dev_max`).

The reference bag is a TA run of `lab2_uturn` on the same car; keep it on that car
at `~/lab2/reference/uturn`, the path the student spec gives.
