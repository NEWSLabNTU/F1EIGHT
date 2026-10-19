#!/usr/bin/env bash
# `just aruco-map CMD`: map a hall's ArUco boards with the car's ZED, and check
# the ArUco localization on the map. Lab-independent: settings come from the
# command line only, and nothing here drives the car.
#
#   survey ORIGIN_ID [DIR] [ARGS]  scan, Ctrl-C solves: a map in the ORIGIN BOARD's frame.
#                                  With --route NAME: then align it to that route (below)
#   scan   ORIGIN_ID [DIR] [ARGS]  the same as survey without --route (DIR ORIGIN_ID works too)
#   align  [DIR] --route NAME      optional: put the map in a route's start-box frame
#                                  from one straight drive from the start box, Ctrl-C
#   solve  [DIR] [ARGS]            solve the scan again, from DIR/survey_data.npz, no car
#   check  [DIR] [ARGS]            drive by hand, watch the localization on the map
#   save                           solve and write the running scan now, without stopping it
#
# DIR: survey and scan write a new surveys/<date>_<time> unless one is named, and
# point surveys/latest at it; align, solve and check read surveys/latest unless
# one is named. Relative paths are from the repository root.
#
# ARGS: key:=value. vehicle_id:= (default sim, the nominal car: it only picks the
# camera mount and IMU calibration, individual_params/config/<id>), dictionary:=
# (default DICT_5X5_1000) and marker_size:= (default 0.38 m, the black square)
# for the scan, and any other argument of the launch file the command runs.
# align takes --route, --force, --vehicle-id (or vehicle_id:=), --bench --truth DIR.
#
# After a Ctrl-C, just (1.58) always ends with "error: interrupted by SIGINT":
# the line before it is this script's verdict.
set -u
cd "$(dirname "$0")/../.."
WEB_ADDR=${ARUCO_MAP_WEB_ADDR:-${LAB2_WEB_ADDR:-0.0.0.0:8081}}
SIM_VEHICLE_ID=sim

die() { echo "aruco-map: $*" >&2; exit 2; }

# A directory argument, or surveys/latest; the rest of "$@" is left in ARGS
read_dir() {
    DIR=surveys/latest; ARGS=("$@")
    if [ $# -gt 0 ] && [[ $1 != -* && $1 != *:=* ]]; then DIR=$1; ARGS=("${@:2}"); fi
    [ -d "$DIR" ] || die "no survey directory '$DIR': run \`just aruco-map survey ORIGIN_ID\` first, or name one"
    DIR=$(realpath "$DIR")
}

# ORIGIN_ID [DIR] or DIR ORIGIN_ID: sets DIR (created, surveys/latest -> it), ORIGIN, ARGS
new_survey_dir() {
    local dir=
    if [[ ${1-} =~ ^[0-9]+$ ]]; then
        ORIGIN=$1; shift
        if [ $# -gt 0 ] && [[ $1 != -* && $1 != *:=* ]]; then dir=$1; shift; fi
    elif [ $# -ge 2 ] && [[ ${1-} != -* && ${1-} != *:=* ]]; then
        dir=$1; ORIGIN=$2; shift 2
    else
        die "usage: just aruco-map $CMD ORIGIN_ID [DIR] [key:=value ...]"
    fi
    [[ $ORIGIN =~ ^[0-9]+$ ]] || die "ORIGIN_ID must be a marker id, got '$ORIGIN'"
    if [ -n "$dir" ]; then DIR=$(realpath -m "$dir"); else DIR=$PWD/surveys/$(date +%Y-%m-%d_%H-%M-%S); fi
    ARGS=("$@")
    mkdir -p "$DIR" surveys
    local target=$DIR
    case "$DIR" in "$PWD"/surveys/*) target=${DIR#"$PWD"/surveys/} ;; esac
    ln -sfn "$target" surveys/latest
    echo "survey -> $DIR (surveys/latest)"
}

# key:=value arguments with vehicle_id:= defaulted: sets LAUNCH_ARGS
launch_args() {
    LAUNCH_ARGS=()
    local a vid=
    for a in "$@"; do
        case "$a" in
            vehicle_id:=*) vid=${a#vehicle_id:=} ;;
            *:=*) LAUNCH_ARGS+=("$a") ;;
            *) die "unexpected argument '$a' (launch arguments are key:=value)" ;;
        esac
    done
    if [ -z "$vid" ]; then
        vid=$SIM_VEHICLE_ID
        echo -e "\033[2mvehicle_id $vid (the nominal car; vehicle_id:=car_X for a car's own camera mount)\033[0m" >&2
    fi
    LAUNCH_ARGS+=("vehicle_id:=$vid")
}

# The scan on DIR / ORIGIN with ARGS (key:=value launch arguments). The marker
# survey node solves and writes the map when it is stopped, so Ctrl-C ends it.
run_scan() {
    launch_args "${ARGS[@]}"
    # The ZED never gets a DISPLAY (camera_display.sh says why); viewers get it back.
    source scripts/camera_display.sh
    camera_display
    local viewer=false; [ -n "$VIEWER_DISPLAY" ] && viewer=true
    local cmd=(play_launch launch --web-addr "$WEB_ADDR" autosdv_marker_survey marker_survey.launch.xml
               "course_dir:=$DIR" "origin_id:=$ORIGIN" "viewer:=$viewer"
               "viewer_display:=$VIEWER_DISPLAY" "${LAUNCH_ARGS[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    (source scripts/env.sh && exec "${cmd[@]}")
    finish_scan
}

# The survey node writes DIR/survey_data.npz on Ctrl-C, then solves. play_launch
# (its Rust parser ignores a node's sigterm_timeout) kills a node 7 s after
# shutdown began, which a long recording's bundle adjustment outlasts: the map
# is then missing, or older than the recording. Solve it here instead.
finish_scan() {
    local npz=$DIR/survey_data.npz
    [ -f "$npz" ] || return 0
    [ -f "$DIR/course.yaml" ] && [ "$DIR/course.yaml" -nt "$npz" ] && return 0
    echo "aruco-map: the survey node stopped before it had written the map; solving $npz" >&2
    local a extra=()
    for a in "${ARGS[@]}"; do
        case "$a" in floor_offset:=*) extra+=(--floor-offset "${a#floor_offset:=}") ;; esac
    done
    (trap - INT; source scripts/env.sh && exec ros2 run autosdv_marker_survey marker_survey_solve "$npz" --course "$DIR" "${extra[@]}")
}

# The alignment on DIR with ARGS (survey_align.py's options); Ctrl-C ends the drive
run_align() {
    (source scripts/camera_display.sh
     camera_display >/dev/null 2>&1      # the ZED: no DISPLAY, no VirtualGL faker
     source scripts/env.sh
     exec python3 scripts/aruco_map/survey_align.py "$DIR" "${ARGS[@]}")
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

# The scan's verdict, the last line before just's "interrupted": 0 when the map
# has two boards or more and the origin among them
scan_verdict() {
    local n has_origin
    read -r n has_origin < <(scan_result "$ORIGIN")
    if [ "$n" -lt 2 ] || [ "$has_origin" != yes ]; then
        echo "aruco-map: the scan found $n board(s), origin $ORIGIN $( [ "$has_origin" = yes ] && echo seen || echo 'NOT seen');" \
             "no usable map. Scan again (\`just aruco-map survey $ORIGIN\`)." >&2
        return 1
    fi
    echo "aruco-map: $n boards, origin board $ORIGIN -> $DIR (frame: the origin board's; survey_report.yaml has the quality)"
    return 0
}

CMD=${1-}; [ $# -gt 0 ] && shift
case "$CMD" in
scan|survey)
    # key:=value arguments go to the scan; --route (or route:=) and the other
    # --options to the alignment, which runs only when a route is named
    pos=(); scan_args=(); align_args=(); align_opts=0; route=
    while [ $# -gt 0 ]; do
        case "$1" in
            route:=*) route=${1#route:=} ;;
            --route) [ $# -ge 2 ] || die "--route needs a value"; route=$2; shift ;;
            --route=*) route=${1#--route=} ;;
            vehicle_id:=*) scan_args+=("$1"); align_args+=("$1") ;;
            *:=*) scan_args+=("$1") ;;
            --truth|--min-distance|--max-lateral|--ready-timeout|--distance|--speed|--bench-timeout|--vehicle-id)
                [ $# -ge 2 ] || die "$1 needs a value"; align_args+=("$1" "$2"); align_opts=1; shift ;;
            -*) align_args+=("$1"); align_opts=1 ;;
            *) pos+=("$1") ;;
        esac
        shift
    done
    [ "$CMD" = scan ] && [ -n "$route" ] && die "scan takes no route: \`just aruco-map survey ORIGIN_ID --route $route\` scans and aligns"
    [ $align_opts = 1 ] && [ -z "$route" ] && die "alignment options need a route: --route NAME"
    new_survey_dir "${pos[@]}" "${scan_args[@]}"
    # Ctrl-C reaches the stage running in the foreground, which ends it; this
    # script only notes it and carries on to the verdict
    trap ':' INT
    stages=1; [ -n "$route" ] && stages=2
    echo
    echo "[1/$stages] Scan: drive or push the car slowly past every board, then press Ctrl-C to solve the map."
    run_scan
    scan_verdict || exit 1
    [ -z "$route" ] && exit 0
    echo
    echo "[2/2] Align to route $route: put the car in the taped start box, facing along the first leg."
    if ! read -r -p "      Press Enter when it is there (q or Ctrl-C to stop here): " answer || [ "${answer-}" = q ]; then
        echo; echo "stopped; the map is kept. Align later: just aruco-map align $DIR --route $route"; exit 0
    fi
    echo "      Wait for READY, drive STRAIGHT along the first leg (2 m or more), stop, press Ctrl-C."
    ARGS=(--route "$route" "${align_args[@]}")
    run_align
    status=$?
    [ $status -eq 0 ] && echo "aruco-map: aligned to $route -> $DIR. Next: just aruco-map check"
    exit $status
    ;;
align)
    read_dir "$@"
    trap ':' INT
    run_align
    ;;
solve)
    read_dir "$@"
    [ -f "$DIR/survey_data.npz" ] || die "$DIR has no survey_data.npz: scan it first"
    source scripts/env.sh
    exec ros2 run autosdv_marker_survey marker_survey_solve "$DIR/survey_data.npz" --course "$DIR" "${ARGS[@]}"
    ;;
check)
    read_dir "$@"
    launch_args "${ARGS[@]}"
    source scripts/camera_display.sh
    camera_display
    rviz=false; [ -n "$VIEWER_DISPLAY" ] && rviz=true
    cmd=(play_launch launch --web-addr "$WEB_ADDR" autosdv_launch aruco_loc_check.launch.xml
         "course_dir:=$DIR" "rviz:=$rviz" "viewer_display:=$VIEWER_DISPLAY" "${LAUNCH_ARGS[@]}")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"
    ;;
save)
    source scripts/env.sh
    exec ros2 service call /marker_survey/save std_srvs/srv/Trigger {}
    ;;
''|-h|--help|help)
    sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'
    ;;
*)
    die "unknown command '$CMD': survey, scan, align, solve, check, save"
    ;;
esac
