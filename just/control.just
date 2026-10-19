# Control system testing.
#
# Recipes here run from the REPO ROOT; see the note in just/tool.just.
set working-directory := '..'

# `just <module>` runs the module's FIRST recipe, so it has to be harmless.
[private]
default:
    @just --list control

# Launch vehicle control test (basic_control.launch.xml); name the car:
#   just control basic car_profile:=car_b
[positional-arguments]
basic *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    play_launch launch control_test basic_control.launch.xml "$@"

# Run trajectory player with straight_10m.yaml (10m straight line)
straight:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    ros2 run control_test trajectory_player --ros-args -p trajectory_file:=straight_10m.yaml

# Run trajectory player with circle.yaml (circular path)
circle:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    ros2 run control_test trajectory_player --ros-args -p trajectory_file:=circle.yaml
