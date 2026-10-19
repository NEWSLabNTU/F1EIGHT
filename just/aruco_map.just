# ArUco map: survey a hall's boards with the car's ZED, align the map to the
# route, and check the localization on it. Nothing here drives the car, so
# `car` (car:= or the lab2 settings file) is optional: without it the nominal
# car's calibration is used. The work is scripts/testing/lab2/aruco_map.sh.
#
#   just aruco-map survey 24             both stages, guided: Ctrl-C ends each stage
#   just aruco-map scan 24               stage 1: drive past every board, Ctrl-C solves
#   just aruco-map align                 stage 2 on surveys/latest: start box, first leg, Ctrl-C
#   just aruco-map solve                 stage 1's solve again, no car
#   just aruco-map check                 drive the course by hand, watch the localization
#
# scan and survey write surveys/<date>_<time> (or a named DIR) and point
# surveys/latest at it; align, solve, check and `just lab2 run` read surveys/latest.

set working-directory := '..'

[private]
default:
    @just --list aruco-map

#   just aruco-map survey 24
#   just aruco-map survey 24 courses/my_hall marker_size:=0.25 --route lab2_uturn
# key:=value arguments go to the scan, --options to the alignment.
# Both stages, guided: scan (Ctrl-C solves), Enter, align (Ctrl-C ends the drive)
[positional-arguments]
survey ORIGIN_ID *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/testing/lab2/aruco_map.sh survey "$@"

#   just aruco-map scan 24                         a new surveys/<date>_<time>, origin board 24
#   just aruco-map scan courses/my_hall 24 dictionary:=ARUCO_MIP_36h12 marker_size:=0.25
# Writes DIR/course.yaml, the map and a report.
# Stage 1: drive or push the car past every board, then Ctrl-C solves the map
[positional-arguments]
scan *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/testing/lab2/aruco_map.sh scan "$@"

#   just aruco-map align                           surveys/latest
#   just aruco-map align surveys/<dir> [--route lab2_uturn] [--force]
#   just aruco-map align DIR --bench --truth TRUTH_DIR      the vehicle bench, no car
# The car in the taped start box; READY, then drive STRAIGHT along the first leg
# (2 m or more), stop, Ctrl-C. The stage-1 result stays in DIR/stage1/.
# Stage 2: put the map in the route's frame (origin at the start box, x along the first leg)
[positional-arguments]
align *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/testing/lab2/aruco_map.sh align "$@"

#   just aruco-map solve                           surveys/latest
#   just aruco-map solve courses/my_hall --origin 3
# Stage 1's solve again from DIR/survey_data.npz, no car
[positional-arguments]
solve *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/testing/lab2/aruco_map.sh solve "$@"

#   just aruco-map check                           surveys/latest
#   just aruco-map check courses/my_hall rviz:=false
# Start with the car in the start box; nothing drives (the actuator is a dry run).
# Localization check: drive the surveyed course BY HAND, watch the trail in RViz
[positional-arguments]
check *ARGS:
    #!/usr/bin/env bash
    # exec: the process just waits on is the one that handles Ctrl-C
    exec scripts/testing/lab2/aruco_map.sh check "$@"
