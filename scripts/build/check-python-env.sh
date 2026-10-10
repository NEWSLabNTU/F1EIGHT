#!/usr/bin/env bash
# Fails fast, with one short message, when a user-site numpy shadows the apt
# python3-numpy that apt's scipy (and colcon-built packages like range_libc)
# were built against. Left uncaught, this surfaces as a multi-page distutils/
# scipy traceback from deep inside a package's build_ext step, which reads
# like a bug in that package rather than a site-packages ordering problem.
set -euo pipefail

# A user-site opencv-python is only a warning: it imports and works against
# the system numpy, but recent wheels declare numpy>=2, so the next pip
# install that resolves dependencies puts a user-site numpy back and the
# check below starts failing.
cv2_file=$(python3 -c "import cv2; print(cv2.__file__)" 2>/dev/null) || cv2_file=""
case "$cv2_file" in
    "$HOME"/.local/*)
        echo "warning: a user-site opencv-python shadows the system OpenCV." >&2
        echo "  found: $cv2_file" >&2
        echo "  its wheel requires numpy>=2, so a later pip install can bring back the" >&2
        echo "  user-site numpy this build refuses. To use the system OpenCV instead:" >&2
        echo "  pip uninstall opencv-python opencv-contrib-python" >&2
        ;;
esac

numpy_file=$(python3 -c "
import sys
try:
    import numpy
except ImportError:
    sys.exit(1)
print(numpy.__file__)
" 2>/dev/null) || exit 0

case "$numpy_file" in
    "$HOME"/.local/*)
        echo "error: a user-site numpy shadows the system python3-numpy package." >&2
        echo "  found: $numpy_file" >&2
        echo "  this breaks colcon builds that link against apt scipy/ROS (e.g. range_libc)." >&2
        echo "  fix: pip uninstall numpy   (repeat until none remain under ~/.local)" >&2
        exit 1
        ;;
esac
