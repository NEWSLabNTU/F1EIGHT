#!/usr/bin/env python3
"""Regenerate the lab2_uturn course from its U-turn radius (and leg length).

    python3 scripts/testing/lab2/make_uturn.py --radius 1.1 [--leg 2] [--write]
    just lab2 make-uturn 1.1 [--leg 2] [--write]

Rewrites path.csv, course.yaml (start pose, boards, the generated header),
lanelet2_map.osm, ar_tag.param.yaml and map_projector_info.yaml in
src/lab/autosdv_waypoint_lab/config/courses/lab2_uturn, after placing the
boards with the coverage search. Without --write it is a dry run: the summary,
a block for the spec and a diff. The work is autosdv_waypoint_lab/uturn.py;
this wrapper runs it from the source tree, no ROS or build needed.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'src/lab/autosdv_waypoint_lab'))

from autosdv_waypoint_lab.uturn import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
