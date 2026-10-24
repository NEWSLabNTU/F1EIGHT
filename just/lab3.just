# Lab 3, indoor map building and NDT localization: the mapping drive, GLIM, the anchor
# to the reflective board, the lanelet2 loop, and the autonomous laps.
#
# Week 1, in the hall (two terminals on the car):
#
#   just lab3 stack car_b                      terminal 1: sensors, board detector, dry-run actuator
#   just lab3 record --car car_b --team 3      terminal 2: Ctrl-C stops and checks the bag
#
# Week 1, on the car's Orin, then anywhere:
#
#   just lab3 map BAG                          GLIM -> BAG_glim/raw_map.pcd
#   just lab3 anchor BAG_glim/raw_map.pcd ~/lab3/team3_map --car car_b
#   just lab3 downsample ~/lab3/team3_map --voxel 0.1
#   (Vector Map Builder: pointcloud_map.pcd as the overlay; draw the clockwise loop;
#    export lanelet2_map.osm into the folder)
#   just lab3 check-lanelet ~/lab3/team3_map   the requirements, then a planning-simulator lap
#
# Week 2, in the hall:
#
#   just lab3 run car_b ~/lab3/team3_map       terminal 1: the full stack; the board sets the pose
#   just lab3 record --car car_b --team 3 --kind run --map ~/lab3/team3_map    terminal 2
#   just lab3 route ~/lab3/team3_map --engage --laps 2                         terminal 3
#
# Settings are on the command line; nothing here reads a config file. The design is
# in the course repository, 2026/labs/Lab3/lab3-design.md.

set working-directory := '..'

web_addr := env_var_or_default('LAB3_WEB_ADDR', '0.0.0.0:8081')

[private]
default:
    @just --list lab3

#   just lab3 stack car_b
#   just lab3 stack car_a rviz:=true
# Week 1: the mapping stack (sensors, board detector, actuator as a dry run)
[positional-arguments]
stack VEHICLE_ID *ARGS:
    #!/usr/bin/env bash
    vid=$1; shift
    case "$vid" in car_a|car_b|car_c|sim) ;; *) echo "usage: just lab3 stack VEHICLE_ID [KEY:=VALUE ...] (car_a, car_b, car_c)" >&2; exit 2 ;; esac
    # The ZED never gets a DISPLAY (camera_display.sh says why)
    source scripts/camera_display.sh
    camera_display
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_lab3 lab3_mapping.launch.xml
         "vehicle_id:=$vid" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

#   just lab3 record --car car_b --team 3                   the mapping bag, checked on Ctrl-C
#   just lab3 record --car car_b --team 3 --kind run --map ~/lab3/team3_map
# Record a Lab 3 bag beside the running stack (second terminal)
[positional-arguments]
record *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_record "$@"

#   just lab3 check-bag ~/lab3/bags/team3_map_20261030_101500
# Can GLIM and the anchor use this mapping bag? PASS/FAIL per requirement
[positional-arguments]
check-bag BAG *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_check_bag "$@"

#   just lab3 map BAG                  writes BAG_glim/raw_map.pcd and glim_report.yaml
#   just lab3 map BAG -o DIR --cpu
# Build the point-cloud map with GLIM (headless), keeping intensity
[positional-arguments]
map BAG *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_map "$@"

#   just lab3 anchor RAW_MAP MAP_DIR --car car_b
#   just lab3 anchor RAW_MAP MAP_DIR --car car_b --dump-debug
# Move the map into the board frame (origin on the floor below the board)
[positional-arguments]
anchor RAW_MAP MAP_DIR *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_anchor "$@"

#   just lab3 downsample MAP_DIR --voxel 0.1 [--max-mb 50]
# Downsample the anchored map to the size limit; you choose the voxel
[positional-arguments]
downsample MAP_DIR *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_downsample "$@"

#   just lab3 check-lanelet MAP_DIR             the requirements, then the simulator lap
#   just lab3 check-lanelet MAP_DIR --no-sim    the requirements only
# Check the lanelet2 loop: closed, clockwise, width, clearance; then a simulator lap
[positional-arguments]
check-lanelet MAP_DIR *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_check_lanelet "$@"

#   just lab3 sim-lap MAP_DIR [--timeout S] [--keep-log]
# The planning-simulator lap alone
[positional-arguments]
sim-lap MAP_DIR *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_sim_lap "$@"

#   just lab3 run car_b ~/lab3/team3_map
#   just lab3 run car_b ~/lab3/team3_map pose_initializer:=none rviz:=true
# Week 2: the full stack on the team's map; the board pose initializer sets the start pose
[positional-arguments]
run VEHICLE_ID MAP_DIR *ARGS:
    #!/usr/bin/env bash
    vid=$1; map=$2; shift 2
    case "$vid" in car_a|car_b|car_c|sim) ;; *) echo "usage: just lab3 run VEHICLE_ID MAP_DIR [KEY:=VALUE ...]" >&2; exit 2 ;; esac
    [ -f "$map/pointcloud_map.pcd" ] || { echo "error: $map has no pointcloud_map.pcd (just lab3 downsample)" >&2; exit 2; }
    [ -f "$map/lanelet2_map.osm" ] || { echo "error: $map has no lanelet2_map.osm" >&2; exit 2; }
    [ -f "$map/board_profile.yaml" ] || { echo "error: $map has no board_profile.yaml (just lab3 anchor)" >&2; exit 2; }
    source scripts/camera_display.sh
    camera_display
    cmd=(play_launch launch --web-addr '{{ web_addr }}' autosdv_lab3 lab3_run.launch.xml
         "vehicle_id:=$vid" "map_path:=$(realpath "$map")" "$@")
    printf '\033[2m%s\033[0m\n' "${cmd[*]}"
    source scripts/env.sh
    exec "${cmd[@]}"

#   just lab3 route MAP_DIR                  set the clockwise loop route
#   just lab3 route MAP_DIR --engage --laps 2
# Set the clockwise loop on the running stack; --engage drives it
[positional-arguments]
route MAP_DIR *ARGS:
    #!/usr/bin/env bash
    map=$1; shift
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_route --map "$map" "$@"

#   just lab3 grade --map TEAM_MAP --run RUN_BAG --checkpoints CP_BAG --marks marks.yaml --sheet sheet.yaml
#   just lab3 grade ... --fallback-map ~/lab3/ta_map_robin_w -o report.yaml
# TA: the week-2 field marks from a team's bags (board init, checkpoints, legs, localization)
[positional-arguments]
grade *ARGS:
    #!/usr/bin/env bash
    source scripts/env.sh
    exec ros2 run autosdv_lab3 lab3_grade "$@"
