# Lab 2, ArUco localization and waypoint following: the simulated car, the vehicle
# bench and the vehicle.
#
# The student commands:
#
#   just lab2 sim                  Part 1: your SpeedPID on the three simulated cars, mark / 25,
#                                  curves.png per car (finds ~/lab2/<id>_lab_2_1 itself)
#   just lab2 sim --no-gui         the same without windows
#   just lab2 plot-sim             redraw the latest Part 1 run's plots
#   just lab2 run team:=3          Part 2: team 3's vehicle model, team3_vehicle. On the car:
#                                  stack, preflight, record, engage (asks), stop, grading table,
#                                  audit.png. On a workstation: the same on the vehicle bench,
#                                  plus the gate verdict
#
# The vehicle models (the handout's lab2_sim_vehicle_launch/ and teamN_vehicle_launch/, the
# latter renamed to the team's number) go under src/lab2_vehicles/ (gitignored) and are built
# with the rest of this repository:
#   colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
#       --packages-up-to team3_vehicle_launch lab2_sim_vehicle_launch
#   just lab2 gate team:=3         the vehicle gate: safety on every car, then the route
#
# Everything below is for the TA, or the step-by-step equivalent.
#
# Settings live in ~/lab2/lab2.yaml (`just lab2 init` writes it; config:=FILE picks
# another; $LAB2_HOME moves ~/lab2): car, team, vehicle_model, part1_submission,
# route, reference, bag_dir. Precedence, highest first: key:=value on the command line,
# that file, the built-in defaults (src/lab/autosdv_waypoint_lab/config/lab2_defaults.yaml).
# `just lab2 config` shows the result. The TA sets each car's letter once:
#
#   just lab2 init car:=A          on the car
#   just lab2 audit                the latest bag: verdict, metrics, violations, audit.png
#
# or, step by step in separate terminals:
#
#   just lab2 gate team:=3         workstation: team3_vehicle's safety on every car, then the route
#   just lab2 vehicle team:=3      car: route + team3_vehicle
#   just lab2 stack car_b          car: the same stack without the lab settings (the reference model)
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
#   just lab2 bench vehicle_model:=team3_vehicle
#   just lab2 vehicle route:=hall_demo
#   just lab2 vehicle route:=lab2_uturn team:=3              Part 2: team 3's vehicle model
#   just lab2 vehicle course_dir:=$PWD/courses/my_hall
#
# Each argument reaches play_launch as one word, so quote a value that
# contains spaces.
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
#                                            (a file that exists: only those keys change)
#   just lab2 init config:=FILE              another file
# Write a commented settings template, or set just the given keys in an existing one
[positional-arguments]
init *ARGS:
    #!/usr/bin/env bash
    exec python3 scripts/testing/lab2/lab2_config.py init "$@"

#   just lab2 bench                                  hall_demo, the reference model
#   just lab2 bench vehicle_model:=team3_vehicle     a team's model (built in ~/lab2/ws)
#   just lab2 bench dropout:=0.4 gyro_bias_error_z:=0.01
# The model's vehicle interface (actuator as a dry run), the EKF, the tracker and the MRM
# path all run; the ZED, the AR-tag localizer, velocity_report and the PCA9685
# are replaced (vehicle_bench_plant, virtual_marker_localizer).
# Vehicle bench: the real vehicle stack on this PC, a plant model for the car
[positional-arguments]
bench *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch waypoint_follow_bench.launch.xml "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    exec "${cmd[@]}"

#   just lab2 vehicle                       route and vehicle model from the config
#   just lab2 vehicle route:=hall_demo team:=3      team3_vehicle (vehicle_model:= names any model)
# On the vehicle: ZED + AR-tag localizer + EKF, the course, the vehicle model
[positional-arguments]
vehicle *ARGS:
    #!/usr/bin/env bash
    unset DISPLAY                       # the ZED cannot have one: scripts/testing/lab2/camera_display.sh
    source scripts/env.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config vehicle "$@" || exit 2
    cmd=(play_launch launch --web-addr '{{ web_addr }}'
         autosdv_launch waypoint_follow.launch.xml "${LAB2_ARGS[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    exec "${cmd[@]}"

#   just lab2 stack car_b                                the default course (hall_demo), reference laws
#   just lab2 stack car_a sensor_suite:=vlp16_zed        any preset value overridden
#   just lab2 stack car_b route:=lab2_uturn rviz:=false  any waypoint_follow.launch.xml argument
# VEHICLE_ID picks individual_params/config/<id>/ (calibration, actuator PWM, preset); nothing
# from ~/lab2/lab2.yaml (no team, no car:=); vehicle_model:= runs another model. The ZED gets no DISPLAY
# (scripts/testing/lab2/camera_display.sh); RViz opens on it when it answers (rviz:= to force).
# The Lab 2 stack (waypoint planner + ArUco localization) on one car, without the lab's settings
[positional-arguments]
stack VEHICLE_ID *ARGS:
    #!/usr/bin/env bash
    vid=$1; shift
    case "$vid" in *:=*|-*) echo "usage: just lab2 stack VEHICLE_ID [KEY:=VALUE ...] (car_a, car_b, car_c)" >&2; exit 2 ;; esac
    for a in "$@"; do
        case "$a" in vehicle_id:=*) echo "error: the vehicle_id is the first argument, not $a" >&2; exit 2 ;; esac
    done
    # The ZED never gets a DISPLAY (camera_display.sh says why); viewers get it back.
    source scripts/testing/lab2/camera_display.sh
    lab2_camera_display
    rviz=false; [ -n "$VIEWER_DISPLAY" ] && rviz=true
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_launch waypoint_follow.launch.xml
         "vehicle_id:=$vid" "rviz:=$rviz" "viewer_display:=$VIEWER_DISPLAY" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

#   just lab2 gate                              the reference model (or the config's team / vehicle_model)
#   just lab2 gate team:=3                      team3_vehicle (vehicle_model:= names any model)
#   just lab2 gate team:=3 --cars car_a         the safety half on one car
#   just lab2 gate --only hall_demo ...         another course instead of lab2_uturn
# Safety (RC simulator, every car's ESC: neutral, stop, watchdog, MRM, emergency, no reverse
# arming), then the graded route on the vehicle bench. Pass it before a car run (the TA checks).
# Gate before car time: the vehicle model's safety on every car, then the route; PASS/FAIL
[positional-arguments]
gate *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config gate "$@" || exit 2
    exec python3 scripts/testing/lab2/bench_gate.py "${LAB2_ARGS[@]}"

#   just lab2 gate-selftest                  three broken models (wrong neutral, no watchdog,
#                                            reverse arming) on car_a; each must FAIL its check
# TA: show the vehicle gate refusing deliberately broken models
[positional-arguments]
gate-selftest *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/vehicle_gate_selftest.py "$@"

#   just lab2 sim                                 the graded set: nominal, low_batt, low_mu; mark / 25
#   just lab2 sim --no-gui                        no windows (as rviz:=false gui:=false); also automatic
#                                                 when no X display answers
# The submission: submission:=DIR, else `part1_submission` from ~/lab2/lab2.yaml, else the one
# ~/lab2/*_lab_2_1 holding lab2_speed/ (none or several: it stops and lists them).
# Outputs: tmp/lab2/part1/<submission>/<date>_<time>/<variant>/ (trace.csv, marks.json, run.json,
# curves.png, play_log), final.json beside them; tmp/lab2/part1/<submission>/latest is the newest.
# The windows stay up after the last car until Enter or the dashboard is closed (--no-hold: not).
# TA: submission:=DIR, variant:=low_mu (one car, no mark), --name LABEL (labels the run directory),
# --out DIR (DIR/<variant>/, the old layout), --final (accepted, the default),
#   just lab2 sim --class autosdv_waypoint_lab.reference.speed_pid_part1:SpeedPID \
#       --params "$(...)"                                          TA: a controller by import path
# No submission anywhere: an error listing what was found and the fix.
# ROS_DOMAIN_ID: yours if exported (non-zero), else 1 + uid % 100. Web UI on LAB2_WEB_ADDR
# (default 127.0.0.1:8081 + uid % 100, per user). scripts/testing/lab2/part1_run.py --help: the rest.
# Part 1: your SpeedPID on the three simulated cars, live; graded (mark / 25), plotted
[positional-arguments]
sim *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    source scripts/testing/lab2/lab2_config.sh
    lab2_config sim "$@" || exit 2
    exec python3 scripts/testing/lab2/part1_run.py "${LAB2_ARGS[@]}"

#   just lab2 plot-sim                                                the latest run, every car
#   just lab2 plot-sim tmp/lab2/part1/B12345678_lab_2_1/latest/nominal one car
#   just lab2 plot-sim DIR --t0 15 --t1 23 -o tmp/stop.png            zoom into a window
# `just lab2 sim` already writes curves.png per car; this redraws. No display needed (Agg).
# Part 1: redraw a run's curves.png (speed, PWM, failed checks)
[positional-arguments]
plot-sim *ARGS:
    #!/usr/bin/env bash
    exec python3 scripts/testing/lab2/plot_part1.py "$@"

#   just lab2 make-part1-course                    rewrite config/courses/lab2_part1 (committed)
#   just lab2 make-part1-course --holds 5,4 --out tmp/lab2/part1/course
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

#   just lab2 run team:=3                       team3_vehicle; the car on a car, else the vehicle bench
#   just lab2 run vehicle_id:=car_c             the reference model (no team) on car C
# On the car (a Jetson, /etc/nv_tegra_release, with `car:` set by the TA's `just lab2 init car:=X`):
# stack, preflight, record, engage (asks), stop, audit. On a workstation: the vehicle bench, engaged
# without asking, then the gate verdict (as `just lab2 gate`). Afterwards: the grading table (value,
# the car's reference, marks, validity, total) and audit.png (opened on $DISPLAY or :1 when one
# answers; the path always printed). Over SSH it re-runs itself inside tmux (`tmux attach -t lab2-run`).
# TA:
#   just lab2 run                               vehicle_model / team, car from the config
#   just lab2 run team:=3 --bench | --car       force the bench or the car
#   just lab2 run --bench [--camera mock]       the bench with rendered boards
#   just lab2 run --bench --yes                 engage without asking (bench only; tests)
# Stack output goes to <bag_dir>/run_logs/lab2_uturn_N/, not the terminal. It asks before engaging
# and stops the car (disengage) on an MRM, a disengage, a localization loss or the timeout.
# Ctrl-C at any point: disengage, close the bag, stop the stack.
# One graded run: stack, preflight, record, engage, watch, stop, audit
[positional-arguments]
run *ARGS:
    #!/usr/bin/env bash
    # lab2_run.py keeps DISPLAY for audit.png and takes it away from the stack (the ZED)
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

#   just lab2 integration-test                       the reference model, rendered boards
#   just lab2 integration-test vehicle_model:=team3_vehicle team_number:=3   a team's model
#   just lab2 integration-test --camera virtual      the modelled localizer instead (faster)
# Bags go to tmp/lab2_integration/bags (bag_dir:= to change), car SIM, team 0. PASS needs: the run
# finished, check-bags (bench mode) ACCEPT, grade valid, audit.png, localization vs true pose ok.
# TA: the whole Part 2 pipeline on the bench, no car; PASS/FAIL
[positional-arguments]
integration-test *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec python3 scripts/testing/lab2/lab2_run.py --bench --yes --integration \
        car:=SIM bag_dir:=tmp/lab2_integration/bags "$@"

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

#   just lab2 make-uturn 1.1 --leg 2            summary, the spec's paste block, a diff
#   just lab2 make-uturn 2.0 --leg 8 --write    a new radius and legs: path, boards, map, header
#   ... --tex <course repo>/2026/labs/Lab2/spec/uturn_geometry.tex   the spec's numbers and map
# The boards come from the coverage search; a layout that misses a goal is not written.
# Regenerate the graded course lab2_uturn from its U-turn radius and leg length (dry run without --write)
[positional-arguments]
make-uturn RADIUS *ARGS:
    #!/usr/bin/env bash
    shift
    exec python3 scripts/testing/lab2/make_uturn.py --radius '{{ RADIUS }}' "$@"

#   just lab2 make-straight 3                    summary with the marker coverage, a diff
#   just lab2 make-straight 3 --write            config/courses/lab2_straight_3m
#   just lab2 make-straight 2.5 --speed 0.6 --name lab2_straight_slow --write
# TA/testing, not graded: lab2_uturn's field (start box, boards, frame), driving +x along its
# first leg; run it with `just lab2 run route:=lab2_straight_3m` (bags lab2_straight_3m_N).
# Generate a straight test route on the U-turn's field (dry run without --write)
[positional-arguments]
make-straight LENGTH *ARGS:
    #!/usr/bin/env bash
    shift
    exec python3 scripts/testing/lab2/make_straight.py '{{ LENGTH }}' "$@"

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
    # car (settings file or car:=) -> vehicle_id:=car_<car>; without one, sim (nothing drives)
    source scripts/testing/lab2/lab2_config.sh
    lab2_config loc-check "$@" || exit 2
    # The ZED never gets a DISPLAY (camera_display.sh says why); viewers get it back.
    source scripts/testing/lab2/camera_display.sh
    lab2_camera_display
    rviz=false; [ -n "$VIEWER_DISPLAY" ] && rviz=true
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_launch waypoint_loc_check.launch.xml
         "course_dir:=$dir" "rviz:=$rviz" "viewer_display:=$VIEWER_DISPLAY" "${LAB2_ARGS[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

# --- Steering calibration (car gate): hold a servo offset, fit the ratio from the circles ----
#   just control basic vehicle_id:=car_b     terminal 1: actuator on this car's PWM; no gate, spotter stops
#   just lab2 steer-hold -25                 terminal 2: steer PWM held at init_steer-25, 0.4 m/s
#   (drive a full circle, measure its radius, Ctrl-C, repeat for +-10 +-25 and the window edge)
#   just lab2 steer-fit --vehicle-id car_b -45:+0.9 -25:+1.7 -10:+4.2 10:-4.2 25:-1.7 45:-0.9
# Radius: positive = left turn. steer-hold reads init_steer, min/max_steer and the ratio from
# the running /autosdv/actuator_node, and refuses offsets the servo window would clamp.

# Hold the steering servo at the running actuator's init_steer + OFFSET (Ctrl-C stops)
[positional-arguments]
steer-hold OFFSET *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_waypoint_lab lab2_steer_cal hold "$@"

# Fit tire_angle_to_steer_ratio and the steering lock: --vehicle-id car_X OFFSET:RADIUS ...
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
    # car (settings file or car:=) -> vehicle_id:=car_<car>: the survey's URDF is that car's;
    # without one, sim, the nominal car (no car's calibration is measured yet)
    source scripts/testing/lab2/lab2_config.sh
    lab2_config survey "$@" || exit 2
    # The ZED never gets a DISPLAY (camera_display.sh says why); viewers get it back.
    source scripts/testing/lab2/camera_display.sh
    lab2_camera_display
    viewer=false; [ -n "$VIEWER_DISPLAY" ] && viewer=true   # the detection video, given a display
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_marker_survey marker_survey.launch.xml
         "course_dir:=$dir" "origin_id:=$origin" "viewer:=$viewer" "viewer_display:=$VIEWER_DISPLAY" "${LAB2_ARGS[@]}")
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

#   just lab2 survey-align                       surveys/latest, the settings' car (--vehicle-id car_b: another); the car in the taped start box
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
    unset DISPLAY                       # the ZED cannot have one: scripts/testing/lab2/camera_display.sh
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
