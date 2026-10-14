# Sourced by the `just lab2` recipes (gate, bench, vehicle).
#
#   lab2_apply_submission "$@"     # sets LAB2_ARGS (array) and exports PYTHONPATH
#
# `submission:=DIR` (or `--submission DIR`) among the arguments is replaced by
# speed_controller_class:= and speed_controller_params:= for the student's SpeedPID
# (scripts/testing/lab2/submission.py) and DIR's package goes on PYTHONPATH.
# `team:=DIR` (or `--team DIR`) does the same for a Part 2 team directory, and
# adds tracker_param_file:= for its path_tracker.param.yaml.
# Every other argument passes through unchanged. Source scripts/env.sh first:
# it must not run after this, or it would not know about PYTHONPATH.
lab2_apply_submission() {
    LAB2_ARGS=()
    local sub="" skip="" a mode=()
    for a in "$@"; do
        if [ -n "$skip" ]; then sub="$a"; skip=""; continue; fi
        case "$a" in
            submission:=*) sub="${a#submission:=}"; mode=() ;;
            --submission) skip=1; mode=() ;;
            --submission=*) sub="${a#--submission=}"; mode=() ;;
            team:=*) sub="${a#team:=}"; mode=(--team) ;;
            --team) skip=1; mode=(--team) ;;
            --team=*) sub="${a#--team=}"; mode=(--team) ;;
            *) LAB2_ARGS+=("$a") ;;
        esac
    done
    [ -n "$skip" ] && { echo "--submission/--team needs a directory" >&2; return 2; }
    [ -z "$sub" ] && return 0
    local helper="scripts/testing/lab2/submission.py" pkg line
    pkg="$(python3 "$helper" "$sub" "${mode[@]}" --pythonpath)" || return 2
    export PYTHONPATH="$pkg${PYTHONPATH:+:$PYTHONPATH}"
    while IFS= read -r line; do LAB2_ARGS+=("$line"); done \
        < <(python3 "$helper" "$sub" "${mode[@]}" --launch-args 2>/dev/null)
    printf '\033[2msubmission %s\033[0m\n' "$pkg"
}
