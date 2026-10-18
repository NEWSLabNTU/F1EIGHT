# Development and monitoring tools.
#
# Recipes here run from the REPO ROOT. Without this a module recipe runs with
# its cwd set to just/, and every ./scripts/... and install/setup.bash path in
# the repo breaks with "not found".
set working-directory := '..'

# `just <module>` runs the module's FIRST recipe, so that recipe has to be the
# harmless one. Private keeps it out of `just --list --list-submodules`.
[private]
default:
    @just --list tool

# Launch RViz with AutoSDV configuration
rviz:
    rviz2 -d ./src/launcher/autosdv_launch/rviz/autosdv.rviz

# Launch PlotJuggler for data visualization
plotjuggler:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    ros2 run plotjuggler plotjuggler

# Launch manual keyboard control (a GUI window: needs DISPLAY; limits from control_test/config/keyboard_control.yaml)
controller:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    ros2 launch control_test keyboard_control.launch.xml

# Launch drive monitor TUI (shows pose, speed, component states)
tui:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    python3 ./scripts/testing/drive/run.py

# Launch only the ZED camera node, for testing the camera alone
zed:
    #!/usr/bin/env bash
    source scripts/env.sh && \
    play_launch launch \
        --web-addr 0.0.0.0:8081 \
        zed_wrapper zed_camera.launch.py camera_model:=zedxm
