"""The Lab 3 board profiles find the low board in simulated scans, across the room.

The simulator is reflective_pose_sim's beam model for each LiDAR, the board centred
0.4 m up, the sensor 0.30 m up with the TF at z = 0 (as AutoSDV's calibration has it)
and the height carried by the profile's base_link_height_above_ground. A sim pass
is the floor, not the proof: the intensity threshold and the gates still need a bag
of the real board.
"""

import os

import numpy as np
import pytest

sim = pytest.importorskip('reflective_pose_sim.vlp32_sim')
from reflective_pose_core import detector  # noqa: E402
from reflective_pose_core.config import load_config  # noqa: E402
from reflective_pose_sim import scenes  # noqa: E402

PROFILES = os.path.join(os.path.dirname(__file__), '..', 'config', 'board')
SCANS = {'robin_w': 3, 'vlp16': 10}   # accumulate_scans in detector_<sensor>.param.yaml


def detect(sensor, range_m):
    n = SCANS[sensor]
    params = load_config(os.path.join(PROFILES, f'lab3_{sensor}.yaml'),
                         scan_count=n).runtime_detector
    scene, _ = scenes.tracking_board_scene(range_m=range_m, bearing_deg=0.0, yaw_deg=0.0,
                                           centre_height=0.4, sensor_height=0.30)
    pts, inten = [], []
    for seed in range(n):
        f = sim.to_pointcloud_fields(sim.simulate(scene, sim.SimParams(sensor=sensor, seed=seed)))
        pts.append(np.column_stack([f['x'], f['y'], f['z']]))
        inten.append(np.asarray(f['intensity']))
    return detector.detect_board(np.vstack(pts), np.concatenate(inten),
                                 scenes.transform_base_sensor(0.0, sensor), params,
                                 height_offset=0.30)


@pytest.mark.parametrize('sensor, range_m', [
    ('robin_w', 1.2), ('robin_w', 5.0), ('robin_w', 11.0),
    ('vlp16', 1.2), ('vlp16', 5.0), ('vlp16', 9.0),
])
def test_low_board_detected(sensor, range_m):
    result = detect(sensor, range_m)
    assert result.status == detector.Status.OK, [r.detail for r in result.rejections]
    assert result.detection.confidence >= 0.6


def test_vlp16_range_capped():
    # 16 rings put one or two on the board past 9 m; the profile stops there
    assert detect('vlp16', 11.0).status != detector.Status.OK


@pytest.mark.parametrize('sensor', ['robin_w', 'vlp16'])
def test_profiles_agree_on_the_board(sensor):
    cfg = load_config(os.path.join(PROFILES, f'lab3_{sensor}.yaml'))
    assert cfg.board.pose_in_map[2] == pytest.approx(0.40)
    assert cfg.runtime_detector.board_centre_height == pytest.approx(0.40)
    assert cfg.map_detector.board_centre_height == pytest.approx(0.40)
