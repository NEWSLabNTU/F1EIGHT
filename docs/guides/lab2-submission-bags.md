# Lab 2 submission bags: record, check, grade

Instructor-facing. Code: `autosdv_waypoint_lab` (`record.py`, `bag_meta.py`,
`check_bags.py`, `route_grade.py`, `grading.py`).

## Record

```bash
just lab2 vehicle course:=lab2_straight ...      # the stack first
just lab2 record --car A --team 3 --name straight_1   # then engage; Ctrl-C ends
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
0.8 m/s, fewer than 3 laps on a circle, localized radius error over the gate,
MRM, disengage, never engaged, unfinished run. With several bags: one of each
route, one car, one team. Exit 1 on any rejection.

## Grade

`just lab2 grade --score BAG... [--reference REFBAG...]`. Definitions, the
points curve and every constant are in the `route_grade.py` docstring and
header. Summary: steady window = first 90 % of 0.8 m/s to the start of the stop
ramp; full marks at <= 1.5x the car's reference value, linear to zero at 4x
(reference floored); fixed absolute lines without a reference. Straight 10 / 4 /
6 (speed RMSE, start overshoot, stop error); circles 10 + 2.5 each.

A circle that fails a gate (radius RMS/max > 0.3 m, fewer than 3 laps, MRM) is
**INVALID**: no points, no penalty, the team re-runs it. A circle with no valid
bag by the end of the slot scores 0. A straight that fails a gate scores 0.
