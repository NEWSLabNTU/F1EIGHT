# Sourced by the `just lab2` recipes that read the Lab 2 settings.
#
#   lab2_config RECIPE "$@"      # sets LAB2_ARGS (array); returns 2 on a config error
#
# scripts/testing/lab2/lab2_config.py does the work: it merges the built-in
# defaults, the user's file (~/lab2/lab2.yaml or config:=FILE) and key:=value on
# the command line, prints the recipe's arguments one per line and a dim summary
# on stderr.
lab2_config() {
    local out
    out="$(python3 scripts/testing/lab2/lab2_config.py "$@")" || return 2
    LAB2_ARGS=()
    if [ -n "$out" ]; then mapfile -t LAB2_ARGS <<< "$out"; fi
    return 0
}
