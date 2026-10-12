from autosdv_waypoint_lab.localization_monitor_node import ERROR, OK, deviation_level, fix_level


def test_fix_age_levels_have_no_warn_tier():
    assert fix_level(None, 5.0)[0] == ERROR
    assert fix_level(4.9, 5.0)[0] == OK
    assert fix_level(5.1, 5.0)[0] == ERROR


def test_path_deviation_levels():
    assert deviation_level(None, 0.0, 0.8, 1.0)[0] == ERROR
    assert deviation_level(0.5, 0.1, 0.8, 1.0)[0] == OK
    assert deviation_level(0.9, 0.1, 0.8, 1.0)[0] == ERROR
    assert deviation_level(0.1, 2.0, 0.8, 1.0)[0] == ERROR
