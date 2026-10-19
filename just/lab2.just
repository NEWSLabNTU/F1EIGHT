# Lab 2, ArUco localization and waypoint following: the vehicle bench and the
# vehicle.
#
# Settings live in ~/lab2/lab2.yaml (`just lab2 init` writes it; config:=FILE picks
# another): car, team, team_dir, submission, route, reference, bag_dir. Precedence,
# highest first: key:=value on the command line, that file, the built-in defaults
# (src/lab/autosdv_waypoint_lab/config/lab2_defaults.yaml). `just lab2 config` shows the
# result. With the file filled in, a graded run is ONE command on the car (in tmux):
#
#   just lab2 run                  stack, preflight, record, engage (asks), stop, audit
#   just lab2 audit                the latest bag: verdict, metrics, violations, audit.png
#
# or, step by step in separate terminals:
#
#   just lab2 gate                 workstation: the submission's SpeedPID on the graded route
#   just lab2 vehicle              car: route + team_dir
#   just lab2 preflight            car, second terminal
#   just lab2 record               rosbags/lab2_uturn_1, _2, ...
#   just lab2 engage               (third terminal), then Ctrl-C the recording
#   just lab2 check-bags           the latest bag
#   just lab2 grade                the latest bag against the reference
#   just lab2 plot                 the latest bag in PlotJuggler
#
# Other arguments go to the launch file or the tool, e.g.
#
#   just lab2 bench
#   just lab2 bench tracker_class:=my_lab2.path_tracker:PathTracker
#   just lab2 vehicle route:=hall_demo
#   just lab2 gate submission:=~/lab2/B12345678_lab_2_1     a student's SpeedPID and calibration
#   just lab2 vehicle route:=lab2_uturn team:=~/lab2/team3   Part 2: the team's controller and lookahead
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

#   just lab2 config                         ~/lab2/lab2.yaml over the defaults
#   just lab2 config config:=FILE car:=B     another file, a command-line override
# Show the Lab 2 settings every recipe would use, and where each value came from
[positional-arguments]
config *ARGS:
    #!/usr/bin/env bash
    exec python3 scripts/testing/lab2/lab2_config.py config "$@"

#   just lab2 init                           ~/lab2/lab2.yaml
#   just lab2 init car:=A team:=3            fill those in, leave the rest commented
#   just lab2 init config:=FILE              another file
# Write a commented settings template (never overwrites an existing file)
[positional-arguments]
init *ARGS:
    #!/usr/bin/env bash
    exec python3 scripts/testing/lab2/lab2_config.py init "$@"

#   just lab2 bench                                  hall_demo, reference laws
#   just lab2 bench dropout:=0.4 gyro_bias_error_z:=0.01
#   just lab2 bench tracker_class:=my_lab2.path_tracker:PathTracker
# The actuator (dry run) and its SpeedPID, the EKF, the tracker and the MRM
# path all run; the ZED, the AR-tag localizer, velocity_report and the PCA9685
# are replaced (vehicle_bench_plant, virtual_marker_localizer).
# Vehicle bench: the real vehicle stack on this PC, a plant model for the car
[positional-arguments]
bench *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/submission.sh
    lab2_apply_submission "$@" || exit 2
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch waypoint_follow_bench.launch.xml "${LAB2_ARGS[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    exec "${cmd[@]}"

#   just lab2 vehicle                       route and team_dir from the config
#   just lab2 vehicle route:=hall_demo team:=~/lab2/team3
# team:= / submission:= / speed_controller_class:= on the command line replace team_dir.
# On the vehicle: ZED + AR-tag localizer + EKF, the course, the student laws
[positional-arguments]
vehicle *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/submission.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config vehicle "$@" || exit 2
    lab2_apply_submission "${LAB2_ARGS[@]}" || exit 2
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch waypoint_follow.launch.xml "${LAB2_ARGS[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    exec "${cmd[@]}"

#   just lab2 gate                     the config's submission
#   just lab2 gate submission:=DIR     DIR = your <student_id>_lab_2_1 (or lab2_speed) directory:
#       its lab2_speed/speed_pid.py, config/speed_pid_car.yaml and config/calibration.yaml are
#       loaded for you (same for `bench` and `vehicle`; scripts/testing/lab2/submission.py shows
#       what is passed)
#   just lab2 gate speed_controller_class:=lab2_speed.speed_pid:SpeedPID "speed_controller_params:={kp: 20.0}"
#   just lab2 gate --only hall_demo ...        another course instead of lab2_uturn
# Gate before car time: your SpeedPID on the graded route (lab2_uturn), full stack; PASS/FAIL
[positional-arguments]
gate *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/submission.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config gate "$@" || exit 2
    lab2_apply_submission "${LAB2_ARGS[@]}" || exit 2
    exec python3 scripts/testing/lab2/bench_gate.py "${LAB2_ARGS[@]}"

#   just lab2 sim submission:=~/lab2/B12345678_lab_2_1            one run, nominal car, ~1 min
#   just lab2 sim submission:=DIR --final                          nominal, low_batt, low_mu: the 25-point mark
#   just lab2 sim submission:=DIR variant:=low_mu rviz:=false gui:=false
#   just lab2 sim                                                  `part1_submission` from ~/lab2/lab2.yaml
# No submission anywhere: the STUB runs (it holds neutral), with a warning.
#   just lab2 sim --name kd_off                                    keep this run: tmp/lab2/part1/kd_off/
# Outputs: tmp/lab2/part1/<submission name>/<variant>/ (trace.csv, marks.json, run.json, play_log);
# the next run with the same name overwrites them, so --name LABEL keeps a run.
# ROS_DOMAIN_ID: yours if exported (non-zero), else 1 + uid % 100. Web UI on LAB2_WEB_ADDR
# (default 127.0.0.1:8081 + uid % 100, per user). scripts/testing/lab2/part1_run.py --help: the rest.
# Part 1: your SpeedPID on the simulated car, one straight, live; graded, marks printed
[positional-arguments]
sim *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config sim "$@" || exit 2
    exec python3 scripts/testing/lab2/part1_run.py "${LAB2_ARGS[@]}"

#   just lab2 plot-sim tmp/lab2/part1/B12345678_lab_2_1/nominal     writes .../nominal/curves.png
#   just lab2 plot-sim DIR --t0 17 --t1 32 -o tmp/stepdown.png        zoom into a window
# No display needed (matplotlib Agg): for runs with rviz:=false gui:=false, which save no dashboard.png.
# Part 1: draw a run's trace.csv (speed, PWM, failed checks) to a PNG
[positional-arguments]
plot-sim *ARGS:
    #!/usr/bin/env bash
    exec python3 scripts/testing/lab2/plot_part1.py "$@"

#   just lab2 make-part1-course                    rewrite config/courses/lab2_part1 (committed)
#   just lab2 make-part1-course --estop-offset 12 --out tmp/lab2/part1/course
# Prints the segment table; part1_profile.yaml is what the scenario node and the grader read.
# Regenerate the Part 1 straight (path, map, part1_profile.yaml) from its speed profile
[positional-arguments]
make-part1-course *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    out=(--out src/lab/autosdv_waypoint_lab/config/courses/lab2_part1)
    case " $* " in *" --out "*|*" --out="*) out=() ;; esac
    exec ros2 run autosdv_waypoint_lab lab2_make_part1_course "${out[@]}" "$@"

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

#   just lab2 health                         beside `just lab2 run` / loc-check, in a second tmux window; Ctrl-C ends it
#   just lab2 health --duration 120          two minutes
#   just lab2 health --with-run              file it under <bag_dir>/run_logs/<the run's bag>/
#   just lab2 health --kernel                also one kernel_cpu_report.sh snapshot at the end
# Writes rosbags/health/<date>_<time>/summary.txt + summary.json (and gpu.csv, tegrastats.log on the car).
# Car-day health: topic rates and gaps, CPU per core, memory, GPU, temperatures, throttling; OK/WARN/FAIL
[positional-arguments]
health *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/health.py "$@"

#   just lab2 run                               the car: route, team_dir, car, team from the config
#   just lab2 run car:=B team_dir:=~/lab2/team3 (team:= here is the team NUMBER, as in record)
#   just lab2 run --bench [--camera mock]       the same flow on the vehicle bench
#   just lab2 run --bench --yes                 engage without asking (bench only; tests)
# Stack output goes to <bag_dir>/run_logs/lab2_uturn_N/, not the terminal. It asks before engaging
# and stops the car (disengage) on an MRM, a disengage, a localization loss or the timeout.
# Ctrl-C at any point: disengage, close the bag, stop the stack.
# One graded run: stack, preflight, record, engage, watch, stop, audit
[positional-arguments]
run *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/lab2_run.py "$@"

#   just lab2 audit                          the latest bag, car/team/reference from the config
#   just lab2 audit rosbags/lab2_uturn_3
#   just lab2 audit --show                   also open the PNG (export DISPLAY=:1 on the car, view over RDP)
# Writes BAG/audit.png (speed, the run on the floor, path deviation) and BAG/audit.json.
# Audit a bag: check-bags + grade verdict, every metric, the violations, and a plot
[positional-arguments]
audit *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config audit "$@" || exit 2
    exec ros2 run autosdv_waypoint_lab lab2_audit "${LAB2_ARGS[@]}"

#   just lab2 integration-test                       the reference controller, rendered boards
#   just lab2 integration-test submission:=DIR       a student's SpeedPID (team_dir:=DIR for Part 2)
#   just lab2 integration-test --camera virtual      the modelled localizer instead (faster)
# Bags go to tmp/lab2_integration/bags (bag_dir:= to change), car SIM, team 0. PASS needs: the run
# finished, check-bags (bench mode) ACCEPT, grade valid, audit.png, localization vs true pose ok.
# TA: the whole Part 2 pipeline on the bench, no car; PASS/FAIL
[positional-arguments]
integration-test *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    ctrl=()
    case " $* " in
        *" submission:="*|*" team_dir:="*|*" speed_controller_class:="*) ;;
        *) ctrl=(speed_controller_class:=autosdv_vehicle_interface.speed_controller:AckermannSpeedPID) ;;
    esac
    exec python3 scripts/testing/lab2/lab2_run.py --bench --yes --integration \
        car:=SIM team:=0 bag_dir:=tmp/lab2_integration/bags "${ctrl[@]}" "$@"

# TA: fuzz the Part 2 gates with mutated copies of a valid bag (--gate: also the bench gate)
[positional-arguments]
fuzz *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/fuzz_bags.py "$@"

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

#   just lab2 record                         car and team from the config, rosbags/lab2_uturn_<next>
#   just lab2 record --car A --team 3 [--name uturn_1]
# Start it after `just lab2 vehicle`, before `just lab2 engage`; Ctrl-C ends it. The course is read
# from the running stack. Writes rosbags/lab2_<name>; check it with `just lab2 check-bags`.
# Record the submission bag: the fixed topic list, the compressed camera, course hash, car and team
[positional-arguments]
record *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config record "$@" || exit 2
    exec ros2 run autosdv_waypoint_lab lab2_record "${LAB2_ARGS[@]}"

#   just lab2 check-bags                                   the latest bag, car and team from the config
#   just lab2 check-bags rosbags/lab2_uturn_1 --car A --team 3
# Would the grader accept these bags? Missing topic, modified course, wrong speed, laps, MRM, off the path
[positional-arguments]
check-bags *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config check-bags "$@" || exit 2
    exec ros2 run autosdv_waypoint_lab lab2_check_bags "${LAB2_ARGS[@]}"

#   just lab2 grade                                               score the latest bag against `reference`
#   just lab2 grade BAG...                                        the engineering metrics
#   just lab2 grade --score BAG [--reference REFBAG]              TA scoring of the lab2_uturn bag
# Score the latest run against the reference, or run metrics of named bags
[positional-arguments]
grade *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config grade "$@" || exit 2
    exec ros2 run autosdv_waypoint_lab lab2_grade_run "${LAB2_ARGS[@]}"

# Regenerate a course's lanelet2_map.osm from its course.yaml; prints marker coverage
[positional-arguments]
course-map *DIRS:
    #!/usr/bin/env bash
    source scripts/env.sh
    [ $# -gt 0 ] || set -- src/lab/autosdv_waypoint_lab/config/courses/*/
    exec ros2 run autosdv_waypoint_lab make_course_map "$@"

#   just lab2 make-uturn 1.5                    summary, the spec's paste block, a diff
#   just lab2 make-uturn 2.0 --leg 8 --write    a new radius and legs: path, boards, map, header
#   ... --tex <course repo>/2026/labs/Lab2/spec/uturn_geometry.tex   the spec's numbers and map
# The boards come from the coverage search; a layout that misses a goal is not written.
# Regenerate the graded course lab2_uturn from its U-turn radius and leg length (dry run without --write)
[positional-arguments]
make-uturn RADIUS *ARGS:
    #!/usr/bin/env bash
    shift
    exec python3 scripts/testing/lab2/make_uturn.py --radius '{{ RADIUS }}' "$@"

# Car in the start box, stack up, not engaged: fix quality and per-marker survey check
camera-check COURSE="src/lab/autosdv_waypoint_lab/config/courses/hall_demo" SECONDS="20":
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_waypoint_lab lab2_camera_check '{{ COURSE }}' --duration '{{ SECONDS }}'

#   just lab2 loc-check courses/my_hall       car in the start box; selector on the joystick
# Nothing here can drive the car (no tracker, no engage, the actuator as a dry run).
#   just lab2 loc-check                         surveys/latest, the most recent survey
# Localization check: drive the surveyed course BY HAND, watch the trail in RViz
[positional-arguments]
loc-check *ARGS:
    #!/usr/bin/env bash
    dir=surveys/latest
    if [ $# -gt 0 ] && [[ $1 != -* && $1 != *:=* ]]; then dir=$1; shift; fi
    if [ ! -d "$dir" ]; then
        echo "error: no course directory '$dir': run \`just lab2 survey ORIGIN_ID\` first, or name one" >&2; exit 1
    fi
    dir=$(realpath "$dir")
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
#   just lab2 survey 24                                    a new surveys/<date>_<time>, origin marker 24
#   just lab2 survey courses/my_hall 24                    or name the directory
#   just lab2 survey courses/my_hall 3 dictionary:=ARUCO_MIP_36h12 marker_size:=0.25
# surveys/latest points at the directory of the most recent survey, named or dated, and
# survey-solve and loc-check use it when no directory is given.
# Writes COURSE_DIR (default surveys/<date>_<time>)/course.yaml, the map, and a report.
[positional-arguments]
survey *ARGS:
    #!/usr/bin/env bash
    if [[ ${1-} =~ ^[0-9]+$ ]]; then
        dir=$PWD/surveys/$(date +%Y-%m-%d_%H-%M-%S)
    elif [ $# -ge 2 ]; then
        dir=$(realpath -m "$1"); shift
    else
        echo "usage: just lab2 survey [COURSE_DIR] ORIGIN_ID [ARGS...]" >&2; exit 2
    fi
    origin="$1"; shift
    if ! [[ $origin =~ ^[0-9]+$ ]]; then
        echo "error: ORIGIN_ID must be a marker id, got '$origin'" >&2; exit 2
    fi
    mkdir -p "$dir" surveys
    case "$dir" in "$PWD"/surveys/*) target=${dir#"$PWD"/surveys/} ;; *) target=$dir ;; esac
    ln -sfn "$target" surveys/latest
    echo "survey -> $dir (surveys/latest)"
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

#   just lab2 survey-solve                       surveys/latest
#   just lab2 survey-solve courses/my_hall --origin 3
# Solve a saved survey again (COURSE_DIR/survey_data.npz, default surveys/latest)
[positional-arguments]
survey-solve *ARGS:
    #!/usr/bin/env bash
    dir=surveys/latest
    if [ $# -gt 0 ] && [[ $1 != -* && $1 != *:=* ]]; then dir=$1; shift; fi
    if [ ! -d "$dir" ]; then
        echo "error: no survey directory '$dir': run \`just lab2 survey ORIGIN_ID\` first, or name one" >&2; exit 1
    fi
    source scripts/env.sh
    exec ros2 run autosdv_marker_survey lab2_marker_survey_solve "$dir/survey_data.npz" --course "$dir" "$@"

#   just lab2 survey-align                       surveys/latest; the car in the taped start box
#   just lab2 survey-align surveys/<dir> [--route lab2_uturn] [--force]
#   just lab2 survey-align DIR --bench --truth TRUTH_DIR      the vehicle bench, no car
# READY, then drive the car STRAIGHT along the taped first leg (2 m or more), stop, Ctrl-C.
# Nothing here can drive the car (no tracker, the actuator as a dry run). Writes the aligned
# course (the route's path.csv, the boards in its frame) and align_report.txt into the survey;
# the stage-1 result stays in DIR/stage1/. Then: loc-check, run.
# Survey stage 2: put the survey into the route's frame (origin at the start box, x along the first leg)
[positional-arguments]
survey-align *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/survey_align.py "$@"

#   just lab2 plot                           the latest bag
#   just lab2 plot rosbags/lab2_uturn_1
#   just lab2 plot --dry-run                 print the command, open nothing
# PlotJuggler asks which topics to load: select all (Ctrl+A), then OK.
# Open a bag in PlotJuggler: target vs measured speed, motor PWM, EKF path vs marker fixes
[positional-arguments]
plot *ARGS:
    #!/usr/bin/env bash
    dry=0; args=()
    for a in "$@"; do if [ "$a" = --dry-run ]; then dry=1; else args+=("$a"); fi; done
    source scripts/testing/lab2/lab2_config.sh
    lab2_config plot "${args[@]}" || exit 2
    [ "${#LAB2_ARGS[@]}" = 1 ] || { echo "usage: just lab2 plot [BAG] [config:=FILE] [--dry-run]" >&2; exit 2; }
    bag=$(realpath "${LAB2_ARGS[0]}")
    [ -f "$bag/metadata.yaml" ] || { echo "error: '$bag' is not a bag directory (no metadata.yaml)" >&2; exit 1; }
    # --disable_opengl: under a remote desktop without GL the plots otherwise draw black
    cmd=(ros2 run plotjuggler plotjuggler --nosplash --disable_opengl
         -d "$bag/metadata.yaml" -l src/lab/autosdv_waypoint_lab/config/plotjuggler/lab2_run.xml)
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    [ "$dry" = 1 ] && exit 0
    source scripts/env.sh
    exec "${cmd[@]}"
