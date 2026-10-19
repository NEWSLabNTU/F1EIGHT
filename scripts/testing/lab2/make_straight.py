#!/usr/bin/env python3
"""Generate a straight Lab 2 TEST route on the U-turn's field (TA tooling, not graded).

    python3 scripts/testing/lab2/make_straight.py 3 [--speed 0.8] [--name NAME] [--write]
    just lab2 make-straight 3 [--speed 0.8] [--name NAME] [--write]

Writes src/lab/autosdv_waypoint_lab/config/courses/lab2_straight_<L>m (path,
course.yaml, map, ar_tag params) in lab2_uturn's start-box frame with its boards.
Without --write it is a dry run: the summary with the marker coverage and a diff.
The work is autosdv_waypoint_lab/straight.py; this wrapper runs it from the
source tree, no ROS or build needed.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'src/lab/autosdv_waypoint_lab'))

from autosdv_waypoint_lab.straight import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
