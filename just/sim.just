# Simulation: the planning simulator, rosbag replay, and full scenarios.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    @just --list sim

# These two named the kind of simulation but not the scenario, and both are
# COSS: the map is COSS, and the recording the logging one replays is the COSS
# drive. They now point at the scenario module, which says so.
# just --list shows only the LAST comment line, so the description goes here.
# Deprecated: use `just coss planning-sim`
planning:
    @echo 'just sim planning -> just coss planning-sim (the map is COSS Park).'
    @just coss planning-sim

# ARGS is still honoured, because scripts and roadmaps pass it.
# just --list shows only the LAST comment line, so the description goes here.
# Deprecated: use `just coss logging-sim`
logging ARGS="":
    #!/usr/bin/env bash
    echo 'just sim logging -> just coss logging-sim (it replays the COSS drive).'
    # `just launch ARGS="x:=y"` is a spelling this project documented for years
    # and which silently does nothing: in just, ARGS="..." AFTER a recipe name
    # is a positional value, not a variable assignment, so this parameter
    # receives the literal text `ARGS=x:=y` and hands it to play_launch, which
    # cannot use it. The launch then runs with DEFAULT arguments.
    #
    # That is invisible on a machine with an NVIDIA driver -- the default
    # pose_source:=cuda_ndt loads and works -- and fatal on one without, where
    # it panics with no CUDA library to load. Fail loudly instead.
    case "{{ARGS}}" in
        ARGS=*)
            echo "error: write the arguments positionally, without 'ARGS='." >&2
            echo >&2
            echo "    just RECIPE \"pose_source:=ndt launch_perception:=false\"" >&2
            echo >&2
            echo "ARGS= after the recipe name is passed through as a literal" >&2
            echo "launch argument, and the launch silently uses its defaults." >&2
            exit 2
            ;;
    esac
    if [ -z '{{ARGS}}' ]; then
        exec just coss logging-sim
    fi
    source scripts/env.sh
    args=({{ARGS}})
    [ -n "${DISPLAY:-}" ] || args+=(rviz:=false)
    exec play_launch launch --web-addr 0.0.0.0:8081 \
        autosdv_launch logging_simulation.launch.yaml "${args[@]}"

# Requires the COSS recording; get it with `just coss download-rosbag`.
# just --list shows only the LAST comment line, so the description goes here.
# Run the COSS Park scenario: logging sim + rosbag feed + localization recording
coss-park:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    parallel --line-buffer ::: \
        "just coss logging-sim" \
        "sleep 40 && ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -l -r 1.0" \
        "sleep 45 && ./scripts/rosbag/record_localization.sh"

# The web UI address. Override for a shared host: RC_SIM_WEB_ADDR=127.0.0.1:8092 just sim rc
web_addr := env_var_or_default('RC_SIM_WEB_ADDR', '0.0.0.0:8081')

#   just sim make-track                                  2,6,11 m/s, 3 s holds, a 2.5, d 2.5
#   just sim make-track --speeds 2,6,11 --hold 3 --accel 2.5 --decel 2.5 --out tmp/rc_sim/tracks/mine
# Prints the segment table; refuses a profile the model cannot drive.
# Size and write a straight track (lanelet2 map, point cloud, track.yaml) from a speed profile
[positional-arguments]
make-track *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_rc_sim rc_sim_make_track "$@"

#   just sim grade ~/lab2/B12345678_lab_2_1           every model variant; marks out of 15
#   just sim grade DIR --variant nominal --jobs 4
#   just sim grade DIR --view                         ... then watch the graded run in RViz
# DIR holds speed_controller_params.yaml (the tunable keys only): docs/guides/rc-sim.md.
# Grade a Lab 2 Part 1 submission in the deterministic simulator (headless, no GPU)
[positional-arguments]
grade *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_rc_sim rc_sim_grade "$@"

#   just sim rc                                       the default straight, production coefficients
#   just sim rc submission:=~/lab2/B12345678_lab_2_1  a student's speed_controller_params.yaml
#   just sim rc track:=tmp/rc_sim/tracks/mine variant:=low_mu rviz:=false
# In RViz: 2D Goal Pose at the stop line, then Engage (or run scripts/testing/rc_sim/smoke.py).
# Not graded (the processes race within a tick); `just sim grade` is the grade.
# Interactive RC simulator: Autoware planning-simulator mode + the real vehicle interface + the plant
[positional-arguments]
rc *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    args=()
    for a in "$@"; do
        case "$a" in
            submission:=*)
                p=$(ros2 run autosdv_rc_sim rc_sim_grade "${a#submission:=}" --print-params) || exit 2
                args+=("speed_controller_params:=$p") ;;
            track:=*|map:=*) args+=("track_dir:=$(realpath "${a#*:=}")") ;;
            *) args+=("$a") ;;
        esac
    done
    [ -n "${DISPLAY:-}" ] || args+=(rviz:=false)
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_rc_sim rc_sim.launch.xml "${args[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    exec "${cmd[@]}"
