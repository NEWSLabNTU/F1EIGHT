#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.sh
! rosdep resolve $(rosdep keys -y --from-paths ../src --ignore-src -r) > rosdep.txt

grep -A1 --no-group-separator '^#apt' rosdep.txt | grep -v '^#apt' | sort > apt.txt
grep -A1 --no-group-separator '^#pip' rosdep.txt | grep -v '^#pip' | sort > pip.txt
