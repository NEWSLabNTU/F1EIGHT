# The coach-board pursuit stack (Lab 2 until 2026-10-13; `just lab2` is now
# the waypoint lab): the simulator bench and the vehicle, plus the field
# calibration recipes both labs use. Arguments after the recipe name go to
# the launch file, e.g.
#
#   just coach sim scenario:=dropout
#   just coach sim controller_class:=my_lab2.cruise_controller:CruiseController
#   just coach vehicle lidar_model:=robin-w imu_yaw_sign:=-1.0
#   just coach vehicle 'speed_controller_params:={kp: 15.0, ki: 5.0}'
#
# Each argument reaches play_launch as one word, so quote a value that
# contains spaces, such as a YAML mapping.
#
# Then, in a second terminal: `just coach engage`.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

# The web UI address. Override for a shared host:
#   LAB2_WEB_ADDR=127.0.0.1:8091 just coach sim
web_addr := env_var_or_default('LAB2_WEB_ADDR', '0.0.0.0:8081')

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    @just --list coach

# Part 1: pursuit stack on the planning simulator with a scripted board
[positional-arguments]
sim *ARGS:
    #!/usr/bin/env bash
    args=(map_path:="$PWD/data/COSS-map-planning")
    [ -n "${DISPLAY:-}" ] || args+=(rviz:=false)
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch coach_pursuit_sim.launch.xml "${args[@]}" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# Part 2: pursuit stack on the vehicle (LiDAR detector, dead reckoning)
[positional-arguments]
vehicle *ARGS:
    #!/usr/bin/env bash
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch coach_pursuit.launch.xml "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# Vehicle bench: the real vehicle stack on this PC, a plant model for the car
#   just coach bench                                  default scenario, reference laws
#   just coach bench scenario:=dropout
#   just coach bench speed_controller_class:=my_lab2.speed_pid:SpeedPID 'speed_controller_params:={kp: 20.0}'
#   just coach bench "plant:=$(cat tmp/motor_plant.yaml)"   a fitted motor (fit-motor -o)
# The actuator (dry run), its safety shell and SpeedPID plug-in, the MRM path
# and dead reckoning all run; only the LiDAR, the ZED, velocity_report, the
# PCA9685 and the detector are replaced (vehicle_bench_plant, virtual_coach).
[positional-arguments]
bench *ARGS:
    #!/usr/bin/env bash
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch coach_pursuit_bench.launch.xml "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# Bench checks (field protocol 3.1, checks 1-4) against fresh benches; PASS/FAIL lines
[positional-arguments]
bench-checks *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/bench_checks.py "$@"

# Before engaging: nodes, rates, TF, availability publisher, MRM, mode, graph
[positional-arguments]
preflight *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/preflight.py "$@"

# Engage autonomous mode (the car starts following the board)
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

# Record the topics the Lab 2 grading reads, to rosbags/lab2_<time>
record NAME="":
    #!/usr/bin/env bash
    source scripts/env.sh
    name='{{ NAME }}'
    out="rosbags/lab2_${name:-$(date +%Y%m%d_%H%M%S)}"
    mkdir -p rosbags
    topics=(
        /perception/coach/board /perception/coach/state
        /planning/trajectory /planning/coach_pursuit/debug
        /localization/kinematic_state /localization/acceleration
        /vehicle/status/velocity_status /vehicle/status/steering_status
        /control/trajectory_follower/control_cmd /control/command/control_cmd
        /api/operation_mode/state /system/fail_safe/mrm_state
        /sensing/imu/imu_data /virtual_coach/truth
        /tf /tf_static
    )
    echo "recording to $out"
    exec ros2 bag record -o "$out" "${topics[@]}"

# --- Field calibration (docs/guides/lab2-field-calibration.md) ---------------

# Start the stack without the pursuit planner first:
#   just coach vehicle launch_planner:=false      (or: just coach sim launch_planner:=false)
# then `just coach record-cal`, this recipe, and `just coach engage` to start.
#   just coach speed-steps                         0,0.3,0.6,0 m/s, 4 s each: about 2 m
#   just coach speed-steps 0,0.5,1.0,0.5,1.5,0 4 25  steps, hold [s], max distance [m]
# The default fits a 2-3 m straight and still identifies deadband, gain and
# time constant (checked with `lab2_fit_motor --synthesize`).
#
# Speed step test: scripted target speeds through the normal command path
speed-steps STEPS="0,0.3,0.6,0" HOLD="4.0" MAX_DISTANCE="2.5":
    #!/usr/bin/env bash
    source scripts/env.sh
    steps="[$(python3 -c "print(', '.join(str(float(x)) for x in '{{ STEPS }}'.split(',') if x.strip()))")]"
    # ROS rejects an integer for a double parameter, so `5` becomes `5.0`
    hold=$(python3 -c "print(float('{{ HOLD }}'))")
    max_distance=$(python3 -c "print(float('{{ MAX_DISTANCE }}'))")
    exec ros2 run autosdv_coach_lab speed_step_planner --ros-args \
        -p "steps:=$steps" -p "hold:=$hold" -p "max_distance:=$max_distance"

#   just coach record-part1                       stack, planner, recording, engage, stop, fit
#   just coach record-part1 --name steps1 --steps 0,0.4,0.7,0 --max-distance 3
#   just coach record-part1 --bench --yes         the same on the vehicle bench
# Writes rosbags/lab2cal_<name>/ with motor_plant.yaml and speed_step.csv inside.
#
# The Part 1 speed-step bag from one terminal (the four recipes above, in order)
[positional-arguments]
record-part1 *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    unset DISPLAY                       # the ZED (its IMU) cannot have one: scripts/testing/lab2/camera_display.sh
    exec python3 scripts/testing/lab2/record_part1.py "$@"

# Saves the running actuator's parameters (init_pwm, brake_pwm, min/max_pwm,
# car_profile) as BAG/actuator_params.yaml first, which fit-motor measures the
# deadband and gain from; refuses to record when /autosdv/actuator_node is not
# up within RECORD_CAL_WAIT seconds (default 30).
#
# Record what the motor fit and the braking report read, to rosbags/lab2cal_<time>
record-cal NAME="":
    #!/usr/bin/env bash
    set -o pipefail
    source scripts/env.sh
    name='{{ NAME }}'
    out="rosbags/lab2cal_${name:-$(date +%Y%m%d_%H%M%S)}"
    [ ! -e "$out" ] || { echo "$out exists: pick another NAME" >&2; exit 2; }
    mkdir -p rosbags tmp
    # ros2 bag record refuses an existing directory, so the dump waits in
    # tmp/ and moves in once the recorder has created $out.
    params=$(mktemp tmp/actuator_params.XXXXXX.yaml)
    node=/autosdv/actuator_node
    wait_s="${RECORD_CAL_WAIT:-30}"
    deadline=$((SECONDS + wait_s))
    until ros2 param dump "$node" > "$params" 2>/dev/null && grep -q '^ *init_pwm:' "$params"; do
        if [ "$SECONDS" -ge "$deadline" ]; then
            rm -f "$params"
            echo "record-cal: $node is not up after ${wait_s} s: start the stack (just coach vehicle" \
                 "launch_planner:=false car_profile:=car_X) first. Not recording: fit-motor needs" \
                 "its init_pwm." >&2
            exit 1
        fi
        sleep 1
    done
    for k in init_pwm brake_pwm min_pwm max_pwm car_profile; do
        grep -q "^ *$k:" "$params" || { echo "record-cal: $node has no $k parameter ($params)" >&2; exit 1; }
    done
    echo "actuator: $(grep -E '^ *(car_profile|init_pwm|brake_pwm|min_pwm|max_pwm):' "$params" | tr -s ' ' | tr '\n' ' ')"
    (
        for _ in $(seq 300); do
            if [ -d "$out" ]; then mv "$params" "$out/actuator_params.yaml" && exit 0; fi
            sleep 0.1
        done
        echo "record-cal: $out never appeared; the actuator parameters are left in $params" >&2
    ) &
    topics=(
        /autosdv/actuator_node/debug/pwm_values /autosdv/actuator_node/debug/pid_values
        /autosdv/actuator_node/debug/control_values /autosdv/actuator_node/debug/mode
        /vehicle/status/velocity_status /vehicle/status/steering_status
        /control/command/control_cmd /control/trajectory_follower/control_cmd
        /control/command/emergency_cmd /system/fail_safe/mrm_state
        /api/operation_mode/state /planning/trajectory /planning/coach_pursuit/debug
        /perception/coach/board /perception/coach/state /virtual_coach/truth
        /localization/kinematic_state /localization/acceleration /sensing/imu/imu_data
        /tf /tf_static
    )
    echo "recording to $out"
    exec ros2 bag record -o "$out" "${topics[@]}"

# Record the LiDAR and the detector for the detector report (LIDAR: vlp16 | robin-w)
record-board LIDAR="robin-w" NAME="":
    #!/usr/bin/env bash
    source scripts/env.sh
    case '{{ LIDAR }}' in
        vlp16) cloud=/sensing/lidar/velodyne_points ;;
        robin-w) cloud=/sensing/lidar/iv_points ;;
        *) echo "LIDAR must be vlp16 or robin-w" >&2; exit 2 ;;
    esac
    name='{{ NAME }}'
    out="rosbags/lab2board_{{ LIDAR }}_${name:-$(date +%Y%m%d_%H%M%S)}"
    mkdir -p rosbags
    echo "recording to $out"
    exec ros2 bag record -o "$out" "$cloud" /perception/coach/board \
        /perception/coach/state /localization/kinematic_state /diagnostics /tf /tf_static

# The neutral the fit measures from is the bag's actuator_params.yaml (record-cal);
# an older bag needs --car-profile car_a|car_b|car_c, and --prior '{init_pwm: N}' wins.
#   just coach fit-motor rosbags/lab2cal_steps1 -o tmp/motor_plant.yaml
#   just coach fit-motor rosbags/lab2cal_old --car-profile car_b
#
# Fit the motor plant (MotorPlantParams) to a speed-step bag; -o FILE writes the YAML
[positional-arguments]
fit-motor BAG *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_coach_lab lab2_fit_motor "$@"

# Export a speed-step bag as t,pwm,measured_speed -- the CSV the Lab 2 students fit their
# feed-forward from (course repo: ta/cars/car_tool.py install-fit ships it to the handout)
[positional-arguments]
export-speed-csv BAG *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_coach_lab lab2_export_speed_csv "$@"

# Reaction time and deceleration per stop, and the envelope they support
[positional-arguments]
braking-report BAG *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_coach_lab lab2_braking_report "$@"

# Detection rate per range, time per scan, intensity threshold, board height
[positional-arguments]
detector-report BAG LIDAR *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    bag="$1" lidar="$2"; shift 2
    exec ros2 run reflective_pose_ros board_tracking_report "$bag" --lidar "$lidar" "$@"
