# ArUco map: survey a hall's ArUco boards with the car's ZED, and check the ArUco
# localization on the map. Lab-independent: settings come from the command line
# (vehicle_id:=, dictionary:=, marker_size:=), and nothing here drives the car.
# The work is scripts/aruco_map/aruco_map.sh; the guide is docs/guides/aruco-map.md.
#
#   just aruco-map survey 24   scan, Ctrl-C solves: a map in board 24's frame
#   ... --align                     then align it to the start box (one straight drive)
#   just aruco-map align            align surveys/latest (optional)
#   just aruco-map solve                      solve the scan again, no car
#   just aruco-map check                      drive by hand, watch the localization
#
# survey and scan write surveys/<date>_<time> (or a named DIR) and point
# surveys/latest at it; align, solve and check read surveys/latest. Lab 2's
# `just lab2 survey`, `survey-align` and `loc-check` run these with the lab's settings.

set working-directory := '..'

[private]
default:
    @just --list aruco-map

#   just aruco-map survey 24            a new surveys/<date>_<time>, origin board 24
#   just aruco-map survey 24 courses/my_hall vehicle_id:=car_c marker_size:=0.25
#   just aruco-map survey 24 --align   then align: start box, first leg
# Writes DIR/course.yaml (the boards in the origin board's frame), the lanelet2
# map, ar_tag.param.yaml, survey_report.yaml, survey_data.npz.
# Survey the boards: drive past every board, Ctrl-C solves (--align: then align)
[positional-arguments]
survey ORIGIN_ID *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/aruco_map/aruco_map.sh survey "$@"

#   just aruco-map scan 24 [DIR] [key:=value ...]   (DIR 24 works too)
# The scan alone: survey without --align
[positional-arguments]
scan *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/aruco_map/aruco_map.sh scan "$@"

#   just aruco-map align                                   surveys/latest
#   just aruco-map align surveys/<dir> [--vehicle-id car_c] [--force]
#   just aruco-map align DIR --bench --truth TRUTH_DIR     Lab 2's vehicle bench, no car
# The car in the taped start box; READY, then drive STRAIGHT along the first leg (2 m or
# more), stop, Ctrl-C. No route: every route taped from that box runs on the result.
# The survey's own result stays in DIR/stage1/.
# Optional: put a survey in the start-box frame (origin at the start box, x along the first leg)
[positional-arguments]
align *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/aruco_map/aruco_map.sh align "$@"

#   just aruco-map solve                                   surveys/latest
#   just aruco-map solve courses/my_hall --origin 3
# Solve the scan again from DIR/survey_data.npz, no car
[positional-arguments]
solve *ARGS:
    #!/usr/bin/env bash
    exec scripts/aruco_map/aruco_map.sh solve "$@"

# Solve and write the running scan now, without stopping it
save:
    #!/usr/bin/env bash
    exec scripts/aruco_map/aruco_map.sh save

#   just aruco-map check                                   surveys/latest
#   just aruco-map check courses/my_hall vehicle_id:=car_c rviz:=false
# Start with the car at the map's start_pose; nothing drives (the actuator is a dry run).
# Localization check: drive BY HAND, watch the trail in RViz
[positional-arguments]
check *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/aruco_map/aruco_map.sh check "$@"
