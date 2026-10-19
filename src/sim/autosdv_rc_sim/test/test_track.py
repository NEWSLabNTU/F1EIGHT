"""Track sizing (the spec's table), the target, refusals, brake authority, the written map."""
import filecmp
import xml.etree.ElementTree as ET

import pytest

from autosdv_rc_sim.params import load_vehicle, variant_names
from autosdv_rc_sim.track import (
    check_feasible, default_track_dir, load_track, make_profile, ProfileError, stop_overshoot,
    write_track)

SPEC = dict(speeds=(2.0, 6.0, 11.0), hold=3.0, accel=2.5, decel=2.5, runout=10.0)
# The committed default track: braking at what regeneration gives (track.py docstring)
DEFAULT_TRACK = dict(SPEC, decel=1.5)


def test_the_spec_table():
    prof = make_profile(**SPEC)
    assert [round(seg.length, 2) for seg in prof.segments] == [0.8, 6.0, 6.4, 18.0, 17.0, 33.0, 24.2, 10.0]
    assert [seg.kind for seg in prof.segments] == [
        'accel', 'hold', 'accel', 'hold', 'accel', 'hold', 'brake', 'runout']
    assert prof.length == pytest.approx(115.4, abs=0.01)
    assert prof.stop_line_s == pytest.approx(105.4, abs=0.01)
    assert sum(seg.duration for seg in prof.segments) == pytest.approx(17.8, abs=0.01)
    assert '| **total** | **115.4 m** | **17.8 s** |' in prof.table()


def test_position_based_target():
    prof = make_profile(**SPEC)
    assert prof.v_target(-0.5) == 0.25            # engage velocity before and at the start
    assert prof.v_target(0.0) == 0.25
    assert prof.v_target(0.2) == pytest.approx(1.0)
    assert prof.v_target(3.0) == 2.0
    assert prof.v_target(60.0) == 11.0
    assert prof.v_target(prof.stop_line_s - 1.0) == pytest.approx((2.0 * 2.5) ** 0.5)
    assert prof.v_target(prof.stop_line_s) == 0.0
    assert prof.v_target(prof.length + 5.0) == 0.0


@pytest.mark.parametrize('kwargs, match', [
    (dict(speeds=(2.0, 2.0)), 'rise'),
    (dict(speeds=(2.0, 12.5)), 'hold'),
    (dict(speeds=(2.0, 6.0), accel=4.0), 'grip'),
    (dict(speeds=(2.0,), hold=0.0), 'positive'),
    (dict(speeds=(2.0, float('inf'))), 'finite'),
    (dict(speeds=(2.0,), hold=float('nan')), 'finite'),
    (dict(speeds=(2.0,), runout=float('inf')), 'finite'),
])
def test_refusals(kwargs, match):
    with pytest.raises(ProfileError, match=match):
        check_feasible(make_profile(**kwargs), load_vehicle())


def test_brake_authority_and_overshoot():
    """Review Focus 1: min_pwm 360 is full brake; a mirrored endpoint would be useless."""
    p = load_vehicle()
    prof = make_profile(**SPEC)
    assert stop_overshoot(prof, p) < 1e-9                      # d 2.5: the full brake follows it
    lo = p.reverse_stop_pwm
    mirrored = (lo - p.min_pwm) / (lo - (2 * p.init_pwm - p.max_pwm))   # 5/95 if 270 were full brake
    assert stop_overshoot(prof, p, level=mirrored) > 20.0


def test_feasibility_warns_about_rear_braking():
    """d 2.5 is within the brake of every variant; the spec's d 3.5 is not, and the heavy car is worst."""
    prof = make_profile(**SPEC)
    assert check_feasible(prof, load_vehicle()) == []
    for variant in variant_names():
        assert stop_overshoot(prof, load_vehicle(variant=variant)) < 1e-9, variant
    steep = make_profile(**dict(SPEC, decel=3.5))
    heavy, nominal = load_vehicle(variant='heavy'), load_vehicle()
    warnings = check_feasible(steep, heavy)
    assert len(warnings) == 1 and 'past the stop line' in warnings[0]
    assert stop_overshoot(steep, heavy) > stop_overshoot(steep, nominal) > 1.0


def test_the_written_map(tmp_path):
    prof = make_profile(**SPEC)
    write_track(prof, tmp_path)
    root = ET.parse(tmp_path / 'lanelet2_map.osm').getroot()
    lanelets = [r for r in root.iter('relation')
                if any(t.get('k') == 'type' and t.get('v') == 'lanelet' for t in r.iter('tag'))]
    limits = [float(next(t.get('v') for t in r.iter('tag') if t.get('k') == 'speed_limit'))
              for r in lanelets]
    assert limits == [7.2, 7.2, 7.2, 21.6, 21.6, 39.6, 39.6, 39.6, 39.6]
    xs = [float(t.get('v')) for t in root.iter('tag') if t.get('k') == 'local_x']
    assert min(xs) == -1.0 and max(xs) == pytest.approx(prof.length, abs=1e-4)
    assert load_track(tmp_path) == prof
    assert (tmp_path / 'map_projector_info.yaml').read_text() == 'projector_type: local\n'


def test_the_committed_default_track_is_current(tmp_path):
    write_track(make_profile(**DEFAULT_TRACK), tmp_path)
    committed = default_track_dir()
    for name in ('lanelet2_map.osm', 'pointcloud_map.pcd', 'map_projector_info.yaml', 'track.yaml'):
        assert filecmp.cmp(tmp_path / name, committed / name, shallow=False), name
