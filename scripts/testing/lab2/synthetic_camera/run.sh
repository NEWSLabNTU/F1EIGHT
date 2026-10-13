#!/usr/bin/env bash
# The REAL ar_tag_based_localizer against a rendered camera image of a course.
#
#   scripts/testing/lab2/synthetic_camera/run.sh [COURSE_DIR] X Y YAW [EKF_DX]
#
# Starts the course map loaders and Autoware's ar_tag_based_localizer with the
# lab's parameters, renders what a forward camera at base_link (0.20, 0, 0.25)
# would see from the car pose (X, Y, YAW) -- every marker of the course, in
# the course's dictionary (ARUCO_MIP_36h12 for ALL_DICTS), warped onto its
# surveyed corners -- and publishes it
# with the EKF pose put EKF_DX metres off (default 0.3). Prints the
# localizer's fixes: they should land on (X, Y, YAW), not on the EKF pose.
#
# What it checks that the bench cannot: the OSM corner order, the marker
# axes, the dictionary and the camera frame, end to end through the stock
# node. Measured 2026-10-13 on hall_demo: 8-14 cm and 1.4-1.8 deg from the
# true pose at full resolution. On a heavily loaded host the 7 MB frames
# may not arrive (no fixes, "debug images 0"); RATE=2 or HALF=1 help.
set -u
cd "$(dirname "$0")/../../../.."
here=scripts/testing/lab2/synthetic_camera
course=$PWD/src/lab/autosdv_waypoint_lab/config/courses/hall_demo
if [ -d "${1:-}" ]; then course=$(cd "$1" && pwd); shift; fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-73}
[ "$ROS_DOMAIN_ID" = 0 ] && { echo "refusing ROS_DOMAIN_ID 0 on a shared host"; exit 1; }
source scripts/env.sh >/dev/null 2>&1
mkdir -p tmp
setsid play_launch launch --web-addr 127.0.0.1:${WEB_PORT:-8092} $here/loc.launch.xml \
    course_dir:=$course > tmp/synthetic_camera_launch.log 2>&1 < /dev/null &
L=$!
dict=$(python3 -c "import sys, yaml; d = (yaml.safe_load(open(sys.argv[1])) or {}).get('dictionary', 'ALL_DICTS'); print(d if d.startswith('DICT_') else 'DICT_ARUCO_MIP_36h12')" $course/course.yaml)
setsid python3 $here/fake_camera.py $course/lanelet2_map.osm $dict "$@" \
    > tmp/synthetic_camera.log 2>&1 < /dev/null &
P=$!
trap 'kill -- -$P -$L 2>/dev/null; sleep 3; kill -9 -- -$P -$L 2>/dev/null' EXIT
sleep 8
python3 $here/probe.py
