import math


def test_mock_camera_skips_boards_behind_or_facing_away():
    from autosdv_aruco_localization import mock_camera_node as mc
    from autosdv_aruco_localization.marker_map import Marker
    cam = mc.Camera(960, 600, 403.0)
    ahead = Marker(1, 3.0, 0.0, 0.25, math.pi)       # 3 m ahead, facing the car
    away = Marker(2, 3.0, 0.0, 0.25, 0.0)            # 3 m ahead, facing away
    behind = Marker(3, -3.0, 0.0, 0.25, 0.0)         # behind, facing the car
    pose = (0.0, 0.0, 0.0, 0.0)
    assert mc.board_in_view(cam, pose, ahead, 12.0)
    assert not mc.board_in_view(cam, pose, away, 12.0)
    assert not mc.board_in_view(cam, pose, ahead, 2.0)
    pats = mc.marker_patterns(mc.aruco_dictionary('DICT_5X5_1000'), [1, 2, 3])
    _, drawn = mc.render(cam, pose, [ahead, away, behind], 0.38, pats)
    assert drawn == [1]
