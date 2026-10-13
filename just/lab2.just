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

# Record the topics the Lab 2 grading reads, to rosbags/lab2wp_<time>
record NAME="":
    #!/usr/bin/env bash
    source scripts/env.sh
    name='{{ NAME }}'
    out="rosbags/lab2wp_${name:-$(date +%Y%m%d_%H%M%S)}"
    mkdir -p rosbags
    topics=(
        /planning/waypoint_path_publisher/progress /planning/trajectory
        /control/path_tracker/debug /control/trajectory_follower/control_cmd
        /control/command/control_cmd /vehicle/status/velocity_status
        /vehicle/status/steering_status /localization/kinematic_state
        /localization/pose_estimator/pose_with_covariance
        /localization/twist_estimator/twist_with_covariance
        /api/operation_mode/state /system/fail_safe/mrm_state
        /sensing/imu/imu_data /vehicle_bench_plant/odometry
        /tf /tf_static
    )
    echo "recording to $out"
    exec ros2 bag record -o "$out" "${topics[@]}"

# Run metrics of recorded runs: laps, cross-track, speed tracking, MRM
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
    rviz=false; [ -n "${DISPLAY:-}" ] && rviz=true
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_launch waypoint_loc_check.launch.xml
         "course_dir:=$dir" "rviz:=$rviz" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# --- Marker survey: map the hall's markers with the car's ZED --------------------------------
# Drive or push the car slowly past every marker, then Ctrl-C (or `just lab2 survey-save`):
#   just lab2 survey courses/my_hall 24                    origin marker 24, the LCTK boards
#   just lab2 survey courses/my_hall 3 dictionary:=ARUCO_MIP_36h12 marker_size:=0.25
# Writes COURSE_DIR/course.yaml (markers in the origin marker's frame), the map, and a report.
[positional-arguments]
survey COURSE_DIR ORIGIN_ID *ARGS:
    #!/usr/bin/env bash
    dir=$(realpath -m "$1"); origin="$2"; shift 2
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
