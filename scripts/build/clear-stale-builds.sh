#!/usr/bin/env bash
# Clear a package's build/ and install/ trees when they hold dangling symlinks.
#
# Under --symlink-install, colcon symlinks every source file of a Python package
# into build/<pkg>/. When a file moves to another package (the ArUco launch files
# left autosdv_waypoint_lab for autosdv_aruco_localization), the old symlink stays
# behind, dangling, and the next build of that package fails with
#   error: can't copy 'build/<pkg>/launch/<file>': doesn't exist or not a regular file
# Removing that one package's build and install trees is the fix; this does it
# before `just build`, for exactly the packages that need it.
set -euo pipefail
root=${1:-.}
[ -d "$root/build" ] || exit 0
for dir in "$root"/build/*/; do
    pkg=$(basename "$dir")
    if [ -n "$(find "$dir" -maxdepth 4 -xtype l -print -quit 2>/dev/null)" ]; then
        echo "clearing build/$pkg and install/$pkg: a file it symlinked has moved or gone"
        rm -rf "$root/build/$pkg" "$root/install/$pkg"
    fi
done
