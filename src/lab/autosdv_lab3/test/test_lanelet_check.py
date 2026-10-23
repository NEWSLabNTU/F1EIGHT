"""F5: the lanelet loop check, the shared loop route, and (opt-in) the simulator lap.

The simulator lap launches Autoware: run it with LAB3_SIM=1 and a private ROS_DOMAIN_ID,
    LAB3_SIM=1 ROS_DOMAIN_ID=62 python3 -m pytest test/test_lanelet_check.py -k sim
"""

import math
import os

import pytest

from autosdv_lab3 import lanelet_check, mapdir
from autosdv_lab3.route import (BACKWARD_CLEAR, MapError, find_chain, load_loop,
                                loop_route, read_lanelets)
from fixtures import make_map


def rows(map_dir):
    return {r.requirement: r for r in lanelet_check.static_checks(map_dir)}


@pytest.fixture(scope='module')
def maps(tmp_path_factory):
    base = tmp_path_factory.mktemp('lab3_maps')
    return {v: make_map(base, v) for v in ('good', 'ccw', 'gap', 'desk', 'narrow', 'tight')}


def test_good_passes_every_static_check(maps):
    r = rows(maps['good'])
    assert all(x.ok for x in r.values()), lanelet_check.table(list(r.values()))


@pytest.mark.parametrize('variant, failing', [
    ('ccw', '3 clockwise'),
    ('gap', '2 closed chain'),
    ('desk', '5 clearance'),
    ('narrow', '4 lane width'),
    ('tight', '6 turn radius'),
])
def test_each_variant_fails_its_requirement_only(maps, variant, failing):
    r = rows(maps[variant])
    assert r[failing].ok is False, lanelet_check.table(list(r.values()))
    assert r[failing].reason
    others = [x for k, x in r.items() if k != failing]
    # after a broken chain nothing else can be measured (None), but nothing else fails
    assert all(x.ok in (True, None) for x in others), lanelet_check.table(list(r.values()))


def test_main_records_and_exits(maps, tmp_path):
    assert lanelet_check.main([maps['good'], '--no-sim']) == 0
    meta = mapdir.load_meta(maps['good'])['lanelet_check']
    assert meta['passed'] is True and meta['sim'] is False
    assert lanelet_check.main([maps['ccw'], '--no-sim']) == 1
    assert mapdir.load_meta(maps['ccw'])['lanelet_check']['passed'] is False


def test_projector_must_be_local(maps, tmp_path):
    d = make_map(tmp_path, 'good')
    with open(os.path.join(d, mapdir.PROJECTOR), 'w') as f:
        f.write('projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: 54SUE\n')
    assert rows(d)['1 map check'].ok is False


def test_chain_order_and_gap(maps):
    lanelets = read_lanelets(os.path.join(maps['good'], mapdir.LANELET))
    order, problem = find_chain(lanelets)
    assert problem == '' and len(order) == 4 and order[0] == min(lanelets)
    with pytest.raises(MapError, match='no successor'):
        load_loop(maps['gap'])


def test_turning_radius_from_vehicle_info():
    v = lanelet_check.vehicle_info()
    assert lanelet_check.min_turning_radius() == pytest.approx(
        v['wheel_base'] / math.tan(v['max_steer_angle']))


def _forward(loop, a, b):
    return (b - a) % loop.length


@pytest.mark.parametrize('variant', ['good', 'ccw'])
def test_route_goes_all_the_way_round(maps, variant):
    loop = load_loop(maps[variant])
    route = loop_route(loop)
    s0 = route.s_start
    # the goal lies just behind the start: nearly a full lap forward
    lap = _forward(loop, s0, route.legs[-1].s_goal)
    assert loop.length - 3.0 < lap < loop.length - 2.0
    # waypoints and leg goals strictly in driving order round the loop
    marks = []
    for leg in route.legs:
        marks += leg.s_waypoints + [leg.s_goal]
    ahead = [_forward(loop, s0, s) for s in marks]
    assert ahead == sorted(ahead) and len(set(ahead)) == len(ahead)
    assert max(b - a for a, b in zip([0.0] + ahead, ahead)) < 0.5 * loop.length
    # Autoware's limits: no leg starts and ends in one lanelet; every leg's goal is
    # more than the behavior path's backward length behind its own start
    starts = [s0] + [leg.s_goal for leg in route.legs[:-1]]
    for a, leg in zip(starts, route.legs):
        assert loop.lane_at(a) != loop.lane_at(leg.s_goal)
        assert loop.length - _forward(loop, a, leg.s_goal) > BACKWARD_CLEAR
    # each lanelet-junction-adjacent mark is moved off the junction
    for s in marks:
        assert min(abs(s - j) for j in list(loop.starts.values()) + [loop.length]) >= 0.3 - 1e-9


def test_route_from_any_pose(maps):
    loop = load_loop(maps['good'])
    for s in (0.0, 3.3, 7.9, 12.0, 18.5):
        x, y, yaw = loop.pose_at(s)
        route = loop_route(loop, start=(x, y, yaw))
        assert route.s_start == pytest.approx(s % loop.length, abs=0.05)
        assert loop.length - 3.0 < _forward(loop, route.s_start, route.legs[-1].s_goal)


# ------------------------------------------------------------------ the simulator lap

sim = pytest.mark.skipif(os.environ.get('LAB3_SIM') != '1' or os.environ.get('ROS_DOMAIN_ID', '0') == '0',
                         reason='launches Autoware: set LAB3_SIM=1 and a private ROS_DOMAIN_ID')


@sim
def test_sim_lap_good(maps):
    from autosdv_lab3 import sim_lap
    res = sim_lap.run_lap(maps['good'])
    print(res.as_dict())
    assert res.ok, res.as_dict()


@sim
def test_sim_lap_ccw_routes_its_own_way(maps):
    """The planner accepts the counter-clockwise loop's route (it follows the lanelets'
    own direction); the check fails the map at requirement 3 regardless.

    The lap itself does not complete in Autoware 1.5.0: the planning validator stops
    the car on trajectory curvature before it moves, on this map and on its mirror
    image's twin that passes clockwise. Recorded here, not asserted, until that is
    understood."""
    from autosdv_lab3 import sim_lap
    res = sim_lap.run_lap(maps['ccw'])
    print(res.as_dict())
    assert res.reason not in ('no route', 'launch died', 'error'), res.as_dict()
    assert rows(maps['ccw'])['3 clockwise'].ok is False
