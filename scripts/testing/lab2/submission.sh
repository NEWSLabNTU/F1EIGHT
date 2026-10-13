# Sourced by the `just lab2` recipes (gate, bench, vehicle).
#
#   lab2_apply_submission "$@"     # sets LAB2_ARGS (array) and exports PYTHONPATH
#
# `submission:=DIR` (or `--submission DIR`) among the arguments is replaced by
# speed_controller_class:= and speed_controller_params:= for the student's SpeedPID
# (scripts/testing/lab2/submission.py) and DIR's package goes on PYTHONPATH.
# Every other argument passes through unchanged. Source scripts/env.sh first:
# it must not run after this, or it would not know about PYTHONPATH.
lab2_apply_submission() {
    LAB2_ARGS=()
    local sub="" skip=0 a
    for a in "$@"; do
        if [ "$skip" = 1 ]; then sub="$a"; skip=0; continue; fi
        case "$a" in
            submission:=*) sub="${a#submission:=}" ;;
            --submission) skip=1 ;;
            --submission=*) sub="${a#--submission=}" ;;
            *) LAB2_ARGS+=("$a") ;;
        esac
    done
    [ "$skip" = 1 ] && { echo "--submission needs a directory" >&2; return 2; }
    [ -z "$sub" ] && return 0
    local helper="scripts/testing/lab2/submission.py" pkg line
    pkg="$(python3 "$helper" "$sub" --pythonpath)" || return 2
    export PYTHONPATH="$pkg${PYTHONPATH:+:$PYTHONPATH}"
    while IFS= read -r line; do LAB2_ARGS+=("$line"); done \
        < <(python3 "$helper" "$sub" --launch-args 2>/dev/null)
    printf '\033[2msubmission %s\033[0m\n' "$pkg"
}
