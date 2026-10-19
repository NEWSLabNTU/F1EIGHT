# Sourced by the `just lab2` recipes (gate, bench, vehicle).
#
#   lab2_apply_submission "$@"     # sets LAB2_ARGS (array) and exports PYTHONPATH
#
# `submission:=DIR` (or `--submission DIR`) among the arguments is replaced by
# speed_controller_class:= and speed_controller_params:= for the student's SpeedPID
# (scripts/testing/lab2/submission.py) and DIR's package goes on PYTHONPATH.
# `team:=DIR` (or `--team DIR`) does the same for a Part 2 team directory, and
# adds tracker_param_file:= for its path_tracker.param.yaml.
# A car_profile:=CAR (passes through) bounds the PWM offsets by that car's profile;
# without one, car_a's. Every other argument passes through unchanged. Source scripts/env.sh first:
# it must not run after this, or it would not know about PYTHONPATH.
#
#   lab2_apply_submission --part1 "$@"   # `just lab2 sim`: submission:=DIR is a Part 1
#                                        # lab2_speed/ submission for the simulated car
lab2_apply_submission() {
    LAB2_ARGS=()
    local sub="" skip="" a mode=() default_mode=() prof=()
    if [ "${1:-}" = "--part1" ]; then default_mode=(--part1); shift; fi
    mode=("${default_mode[@]}")
    for a in "$@"; do
        if [ -n "$skip" ]; then sub="$a"; skip=""; continue; fi
        case "$a" in
            submission:=*) sub="${a#submission:=}"; mode=("${default_mode[@]}") ;;
            --submission) skip=1; mode=("${default_mode[@]}") ;;
            --submission=*) sub="${a#--submission=}"; mode=("${default_mode[@]}") ;;
            team:=*) sub="${a#team:=}"; mode=(--team) ;;
            --team) skip=1; mode=(--team) ;;
            --team=*) sub="${a#--team=}"; mode=(--team) ;;
            car_profile:=*) prof=(--car-profile "${a#car_profile:=}"); LAB2_ARGS+=("$a") ;;
            *) LAB2_ARGS+=("$a") ;;
        esac
    done
    [ -n "$skip" ] && { echo "--submission/--team needs a directory" >&2; return 2; }
    [ -z "$sub" ] && return 0
    local helper="scripts/testing/lab2/submission.py" pkg line
    pkg="$(python3 "$helper" "$sub" "${mode[@]}" "${prof[@]}" --pythonpath)" || return 2
    export PYTHONPATH="$pkg${PYTHONPATH:+:$PYTHONPATH}"
    while IFS= read -r line; do LAB2_ARGS+=("$line"); done \
        < <(python3 "$helper" "$sub" "${mode[@]}" "${prof[@]}" --launch-args 2>/dev/null)
    printf '\033[2msubmission %s\033[0m\n' "$pkg"
}
