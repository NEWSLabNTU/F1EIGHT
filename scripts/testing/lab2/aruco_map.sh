#!/usr/bin/env bash
# `just aruco-map CMD`: map a hall's ArUco boards with the car's ZED, put the map
# in the route's frame, and check the localization on it.
#
#   survey ORIGIN_ID [DIR] [ARGS]  scan, then align, guided: Ctrl-C ends each stage
#   scan   [DIR] ORIGIN_ID [ARGS]  stage 1: drive past every board, Ctrl-C solves the map
#   align  [DIR] [ARGS]            stage 2: from the start box, drive the first leg, Ctrl-C
#   solve  [DIR] [ARGS]            stage 1's solve again, from DIR/survey_data.npz
#   check  [DIR] [ARGS]            drive the course by hand, watch the localization
#
# DIR: scan and survey write a new surveys/<date>_<time> unless one is named, and
# point surveys/latest at it; align, solve and check read surveys/latest unless
# one is named. `car` (car:= or the lab2 settings file) is optional throughout:
# nothing here drives, and without one the nominal car's calibration (sim) is used.
set -u
cd "$(dirname "$0")/../../.."
WEB_ADDR=${LAB2_WEB_ADDR:-0.0.0.0:8081}

die() { echo "aruco-map: $*" >&2; exit 2; }

# A directory argument, or surveys/latest; the rest of "$@" is left in ARGS
read_dir() {
    DIR=surveys/latest; ARGS=("$@")
    if [ $# -gt 0 ] && [[ $1 != -* && $1 != *:=* ]]; then DIR=$1; ARGS=("${@:2}"); fi
    [ -d "$DIR" ] || die "no survey directory '$DIR': run \`just aruco-map scan ORIGIN_ID\` first, or name one"
    DIR=$(realpath "$DIR")
}

# [DIR] ORIGIN_ID: sets DIR (created, surveys/latest -> it), ORIGIN, ARGS
new_survey_dir() {
    if [[ ${1-} =~ ^[0-9]+$ ]]; then
        DIR=$PWD/surveys/$(date +%Y-%m-%d_%H-%M-%S)
    elif [ $# -ge 2 ]; then
        DIR=$(realpath -m "$1"); shift
    else
        die "usage: just aruco-map $CMD [DIR] ORIGIN_ID [ARGS...]"
    fi
    ORIGIN=$1; shift
    [[ $ORIGIN =~ ^[0-9]+$ ]] || die "ORIGIN_ID must be a marker id, got '$ORIGIN'"
    ARGS=("$@")
    mkdir -p "$DIR" surveys
    local target=$DIR
    case "$DIR" in "$PWD"/surveys/*) target=${DIR#"$PWD"/surveys/} ;; esac
    ln -sfn "$target" surveys/latest
    echo "survey -> $DIR (surveys/latest)"
}

# Stage 1 on DIR / ORIGIN with ARGS (key:=value launch arguments). The marker
# survey node solves and writes the map when it is stopped, so Ctrl-C ends it.
run_scan() {
    # car (settings file or car:=) -> vehicle_id:=car_<car>; without one, sim
    source scripts/testing/lab2/lab2_config.sh
    lab2_config survey "${ARGS[@]}" || exit 2
    # The ZED never gets a DISPLAY (camera_display.sh says why); viewers get it back.
    source scripts/testing/lab2/camera_display.sh
    lab2_camera_display
    local viewer=false; [ -n "$VIEWER_DISPLAY" ] && viewer=true
    local cmd=(play_launch launch --web-addr "$WEB_ADDR" autosdv_marker_survey marker_survey.launch.xml
               "course_dir:=$DIR" "origin_id:=$ORIGIN" "viewer:=$viewer"
               "viewer_display:=$VIEWER_DISPLAY" "${LAB2_ARGS[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    (source scripts/env.sh && exec "${cmd[@]}")
}

# Stage 2 on DIR with ARGS (survey_align.py's options); Ctrl-C ends the drive
run_align() {
    (unset DISPLAY                       # the ZED cannot have one: camera_display.sh
     source scripts/env.sh
     exec python3 scripts/testing/lab2/survey_align.py "$DIR" "${ARGS[@]}")
}

# How many boards DIR/course.yaml holds, and whether the origin is one: "N yes|no"
scan_result() {
    python3 - "$DIR/course.yaml" "$1" <<'EOF'
import sys, yaml
try:
    c = yaml.safe_load(open(sys.argv[1])) or {}
except OSError:
    print('0 no'); sys.exit()
ids = [int(m['id']) for m in c.get('markers') or []]
print(len(ids), 'yes' if int(sys.argv[2]) in ids else 'no')
EOF
}

CMD=${1-}; [ $# -gt 0 ] && shift
case "$CMD" in
scan)
    new_survey_dir "$@"
    echo "Drive or push the car slowly past every board, then press Ctrl-C to solve the map."
    run_scan
    ;;
align)
    read_dir "$@"
    run_align
    ;;
solve)
    read_dir "$@"
    [ -f "$DIR/survey_data.npz" ] || die "$DIR has no survey_data.npz: scan it first"
    source scripts/env.sh
    exec ros2 run autosdv_marker_survey lab2_marker_survey_solve "$DIR/survey_data.npz" --course "$DIR" "${ARGS[@]}"
    ;;
check)
    read_dir "$@"
    # car (settings file or car:=) -> vehicle_id:=car_<car>; without one, sim (nothing drives)
    source scripts/testing/lab2/lab2_config.sh
    lab2_config loc-check "${ARGS[@]}" || exit 2
    source scripts/testing/lab2/camera_display.sh
    lab2_camera_display
    rviz=false; [ -n "$VIEWER_DISPLAY" ] && rviz=true
    cmd=(play_launch launch --web-addr "$WEB_ADDR" autosdv_launch waypoint_loc_check.launch.xml
         "course_dir:=$DIR" "rviz:=$rviz" "viewer_display:=$VIEWER_DISPLAY" "${LAB2_ARGS[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"
    ;;
survey)
    # survey ORIGIN_ID [DIR] [ARGS]: key:=value arguments go to the scan, the
    # others (--route, --force, ...) to the alignment
    [[ ${1-} =~ ^[0-9]+$ ]] || die "usage: just aruco-map survey ORIGIN_ID [DIR] [ARGS...]"
    origin=$1; shift
    dir_arg=(); if [ $# -gt 0 ] && [[ $1 != -* && $1 != *:=* ]]; then dir_arg=("$1"); shift; fi
    scan_args=(); align_args=()
    for a in "$@"; do if [[ $a == *:=* ]]; then scan_args+=("$a"); else align_args+=("$a"); fi; done
    new_survey_dir "${dir_arg[@]}" "$origin" "${scan_args[@]}"
    # Ctrl-C reaches the stage running in the foreground, which ends it; this
    # script only notes it and carries on
    trap ':' INT
    echo
    echo "[1/2] Scan: drive or push the car slowly past every board, then press Ctrl-C."
    run_scan
    read -r n has_origin < <(scan_result "$ORIGIN")
    if [ "$n" -lt 2 ] || [ "$has_origin" != yes ]; then
        echo "aruco-map: the scan found $n board(s), origin $ORIGIN $( [ "$has_origin" = yes ] && echo seen || echo 'NOT seen'):" \
             "not aligning. Scan again (\`just aruco-map survey $ORIGIN\`)." >&2
        exit 1
    fi
    echo "[1/2] done: $n boards -> $DIR"
    echo
    echo "[2/2] Align: put the car in the taped start box, facing along the first leg."
    if ! read -r -p "      Press Enter when it is there (q or Ctrl-C to stop here): " answer || [ "${answer-}" = q ]; then
        echo; echo "stopped; the scan is kept. Align later: just aruco-map align $DIR"; exit 0
    fi
    echo "      Wait for READY, drive STRAIGHT along the first leg (2 m or more), stop, press Ctrl-C."
    ARGS=("${align_args[@]}")
    run_align
    status=$?
    [ $status -eq 0 ] && echo "Next: just aruco-map check"
    exit $status
    ;;
''|-h|--help|help)
    sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
    ;;
*)
    die "unknown command '$CMD': survey, scan, align, solve, check"
    ;;
esac
