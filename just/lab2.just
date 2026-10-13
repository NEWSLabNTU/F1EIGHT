# Lab 2, ArUco localization and waypoint following: the vehicle bench and the
# vehicle. Arguments after the recipe name go to the launch file, e.g.
#
#   just lab2 bench
#   just lab2 bench tracker_class:=my_lab2.path_tracker:PathTracker
#   just lab2 vehicle course:=hall_demo
#   just lab2 vehicle course_dir:=$PWD/courses/my_hall 'speed_controller_params:={kp: 15.0}'
#
# Each argument reaches play_launch as one word, so quote a value that
# contains spaces, such as a YAML mapping.
#
# Then, in a second terminal: `just lab2 engage`.
#
# The coach-board pursuit lab, and the field calibration recipes (speed steps,
# motor fit, braking), are `just coach`.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

# The web UI address. Override for a shared host:
#   LAB2_WEB_ADDR=127.0.0.1:8091 just lab2 bench
web_addr := env_var_or_default('LAB2_WEB_ADDR', '0.0.0.0:8081')

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    @just --list lab2

# Vehicle bench: the real vehicle stack on this PC, a plant model for the car
#   just lab2 bench                                  hall_demo, reference laws
#   just lab2 bench dropout:=0.4 gyro_bias_error_z:=0.01
#   just lab2 bench tracker_class:=my_lab2.path_tracker:PathTracker
# The actuator (dry run) and its SpeedPID, the EKF, the tracker and the MRM
# path all run; the ZED, the AR-tag localizer, velocity_report and the PCA9685
# are replaced (vehicle_bench_plant, virtual_marker_localizer).
[positional-arguments]
bench *ARGS:
    #!/usr/bin/env bash
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch waypoint_follow_bench.launch.xml "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# On the vehicle: ZED + AR-tag localizer + EKF, the course, the student laws
[positional-arguments]
vehicle *ARGS:
    #!/usr/bin/env bash
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch waypoint_follow.launch.xml "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# Gate before car time: your SpeedPID on the three graded routes, full stack; PASS/FAIL per route
#   just lab2 gate speed_controller_class:=lab2_waypoint.speed_pid:SpeedPID "speed_controller_params:={kp: 15.0}"
#   just lab2 gate --only lab2_straight ...
[positional-arguments]
gate *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/bench_gate.py "$@"

# Bench checks on fresh benches (follow, marker loss, path publisher loss, faults); PASS/FAIL lines
[positional-arguments]
bench-checks *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/waypoint_checks.py "$@"

# Before engaging: nodes, rates, marker fixes, start box, TF, availability, MRM, mode, graph
[positional-arguments]
preflight *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/waypoint_preflight.py "$@"

# Engage autonomous mode (the car starts the course)
engage:
    #!/usr/bin/env bash
    source scripts/env.sh
    ros2 service call /api/operation_mode/change_to_autonomous \
        autoware_adapi_v1_msgs/srv/ChangeOperationMode {}

# Leave autonomous mode (the car stops under Autoware's stop mode)
disengage:
    #!/usr/bin/env bash
    source scripts/env.sh
    ros2 service call /api/operation_mode/change_to_stop \
        autoware_adapi_v1_msgs/srv/ChangeOperationMode {}

# Start a new run from where the car is (resets laps), without relaunching
restart-run:
    #!/usr/bin/env bash
    source scripts/env.sh
    ros2 service call /planning/waypoint_path_publisher/restart std_srvs/srv/Trigger {}

# Re-seed the EKF at the start box (car put back in the box)
reinit:
    #!/usr/bin/env bash
    source scripts/env.sh
    ros2 service call /localization/pose_twist_fusion_filter/start_pose_init/reinit \
        std_srvs/srv/Trigger {}

# Record the submission bag: the fixed topic list, the compressed camera, course hash, car and team
#   just lab2 record --car A --team 3 [--name straight_1]
# Start it after `just lab2 vehicle`, before `just lab2 engage`; Ctrl-C ends it. The course is read
# from the running stack. Writes rosbags/lab2_<name>; check it with `just lab2 check-bags`.
[positional-arguments]
record *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_waypoint_lab lab2_record "$@"

# Would the grader accept these bags? Missing topic, modified course, wrong speed, laps, MRM
#   just lab2 check-bags rosbags/lab2_straight rosbags/lab2_cw rosbags/lab2_ccw --car A --team 3
[positional-arguments]
check-bags *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_waypoint_lab lab2_check_bags "$@"

# Run metrics of recorded runs: laps, cross-track, speed tracking, MRM
#   just lab2 grade BAG...                                        the engineering metrics
#   just lab2 grade --score BAG... --reference REFBAG...          TA scoring of the route bags
[positional-arguments]
grade *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_waypoint_lab lab2_grade_run "$@"

# Regenerate a course's lanelet2_map.osm from its course.yaml; prints marker coverage
[positional-arguments]
course-map *DIRS:
    #!/usr/bin/env bash
    source scripts/env.sh
    [ $# -gt 0 ] || set -- src/lab/autosdv_waypoint_lab/config/courses/*/
    exec ros2 run autosdv_waypoint_lab make_course_map "$@"

# Car in the start box, stack up, not engaged: fix quality and per-marker survey check
camera-check COURSE="src/lab/autosdv_waypoint_lab/config/courses/hall_demo" SECONDS="20":
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_waypoint_lab lab2_camera_check '{{ COURSE }}' --duration '{{ SECONDS }}'

#   just lab2 loc-check courses/my_hall       car in the start box; selector on the joystick
# Nothing here can drive the car (no tracker, no engage, the actuator as a dry run).
# Localization check: drive the surveyed course BY HAND, watch the trail in RViz
[positional-arguments]
loc-check COURSE_DIR *ARGS:
    #!/usr/bin/env bash
    dir=$(realpath -m "$1"); shift
    # DISPLAY is usable only if an X connection works. An empty or dead DISPLAY makes Argus
    # fail and the ZED wrapper hang at "CAMERA OPENING": unset it (never leave it empty).
    if [ "${DISPLAY+set}" = set ]; then
        ok=0
        if [ -z "$DISPLAY" ]; then ok=1
        elif command -v xdpyinfo >/dev/null 2>&1; then xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || ok=1
        elif command -v xset >/dev/null 2>&1; then xset -display "$DISPLAY" q >/dev/null 2>&1 || ok=1
        else
            case "$DISPLAY" in
                :*|unix:*) n=${DISPLAY#*:}; n=${n%%.*}; [ -S "/tmp/.X11-unix/X$n" ] || ok=1 ;;
            esac
        fi
        if [ "$ok" != 0 ]; then
            echo "warning: DISPLAY='$DISPLAY' has no working X server (a desktop is usually :1 on the car); unsetting it for this launch, no viewer." >&2
            unset DISPLAY
        fi
    fi
    rviz=false; [ -n "${DISPLAY:-}" ] && rviz=true
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_launch waypoint_loc_check.launch.xml
         "course_dir:=$dir" "rviz:=$rviz" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# --- Steering calibration (car gate): hold a servo offset, fit the ratio from the circles ----
#   just control basic                       terminal 1 (no gate: the spotter is the stop)
#   just lab2 steer-hold -25                 terminal 2: steer PWM held at 489-25, 0.4 m/s
#   (drive a full circle, measure its radius, Ctrl-C, repeat for +-10 +-25 +-50)
#   just lab2 steer-fit -50:+1.6 -25:+3.1 -10:+7.9 10:-7.9 25:-3.1 50:-1.6
# Radius: positive = left turn. Pass --ratio R if the actuator runs a ratio other than -1000.

# Publish the tire angle that holds the steering servo at 489 + OFFSET (Ctrl-C stops)
[positional-arguments]
steer-hold OFFSET *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_waypoint_lab lab2_steer_cal hold "$@"

# Fit tire_angle_to_steer_ratio and the true steering lock from OFFSET:RADIUS pairs
[positional-arguments]
steer-fit *PAIRS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_waypoint_lab lab2_steer_cal fit "$@"

# --- Marker survey: map the hall's markers with the car's ZED --------------------------------
# Drive or push the car slowly past every marker, then Ctrl-C (or `just lab2 survey-save`):
#   just lab2 survey courses/my_hall 24                    origin marker 24, the lab's 0.50 m boards, marker_size 0.38
#   just lab2 survey courses/my_hall 3 dictionary:=ARUCO_MIP_36h12 marker_size:=0.25
# Writes COURSE_DIR/course.yaml (markers in the origin marker's frame), the map, and a report.
[positional-arguments]
survey COURSE_DIR ORIGIN_ID *ARGS:
    #!/usr/bin/env bash
    dir=$(realpath -m "$1"); origin="$2"; shift 2
    # DISPLAY is usable only if an X connection works. An empty or dead DISPLAY makes Argus
    # fail and the ZED wrapper hang at "CAMERA OPENING": unset it (never leave it empty).
    if [ "${DISPLAY+set}" = set ]; then
        ok=0
        if [ -z "$DISPLAY" ]; then ok=1
        elif command -v xdpyinfo >/dev/null 2>&1; then xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || ok=1
        elif command -v xset >/dev/null 2>&1; then xset -display "$DISPLAY" q >/dev/null 2>&1 || ok=1
        else
            case "$DISPLAY" in
                :*|unix:*) n=${DISPLAY#*:}; n=${n%%.*}; [ -S "/tmp/.X11-unix/X$n" ] || ok=1 ;;
            esac
        fi
        if [ "$ok" != 0 ]; then
            echo "warning: DISPLAY='$DISPLAY' has no working X server (a desktop is usually :1 on the car); unsetting it for this launch, no viewer." >&2
            unset DISPLAY
        fi
    fi
    viewer=false; [ -n "${DISPLAY:-}" ] && viewer=true      # the detection video, given a display
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_marker_survey marker_survey.launch.xml
         "course_dir:=$dir" "origin_id:=$origin" "viewer:=$viewer" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# Solve and write the survey now, without stopping it
survey-save:
    #!/usr/bin/env bash
    source scripts/env.sh
    ros2 service call /marker_survey/save std_srvs/srv/Trigger {}

# Solve a saved survey again (COURSE_DIR/survey_data.npz), e.g. with another origin
[positional-arguments]
survey-solve COURSE_DIR *ARGS:
    #!/usr/bin/env bash
    dir="$1"; shift
    source scripts/env.sh
    exec ros2 run autosdv_marker_survey lab2_marker_survey_solve "$dir/survey_data.npz" --course "$dir" "$@"
