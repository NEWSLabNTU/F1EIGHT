"""vehicle.yaml: complete, every value tagged, config values equal to their files."""
import dataclasses
import math

from conftest import REPO
import pytest
import yaml

from autosdv_rc_sim.params import (
    DEFAULT_FILE, GRAVITY, load_tagged, load_vehicle, SOURCES, VehicleParams, variant_names)

PKG_DIRS = {
    'autosdv_vehicle_description': REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_description',
    'autosdv_vehicle_interface': REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface',
    'autosdv_rc_sim': REPO / 'src/sim/autosdv_rc_sim',
    'individual_params': REPO / 'src/param/autoware_individual_params/individual_params',
}


def test_every_field_is_present_and_tagged():
    tagged = load_tagged()
    names = {f.name for f in dataclasses.fields(VehicleParams)}
    assert set(tagged.entries) == names
    for key, entry in tagged.entries.items():
        assert entry['source'] in SOURCES, key


def test_config_values_match_their_files():
    checked = 0
    for key, entry in load_tagged().entries.items():
        if entry['source'] != 'config':
            continue
        ref, _, param = entry['from'].rpartition(':')
        pkg, _, rel = ref.partition('/')
        with open(PKG_DIRS[pkg] / rel) as f:
            ros = yaml.safe_load(f)['/**']['ros__parameters']
        assert ros[param] * entry.get('scale', 1.0) == pytest.approx(entry['value'], rel=1e-12), key
        checked += 1
    assert checked >= 13


def test_derived_values():
    p = load_vehicle()
    assert p.lr == pytest.approx(0.169)
    assert p.kt == pytest.approx(60.0 / (2.0 * math.pi * 3000.0)) == p.ke
    assert p.rear_axle_inertia == pytest.approx(1.4e-4 + 1.0e-6 * 14.5 * 14.5)
    assert p.fz_front_static + p.fz_rear_static == pytest.approx(p.mass * GRAVITY)
    assert p.b_lat_front * p.pacejka_c * p.mu * p.fz_front_static == pytest.approx(94.0)


def test_motor_braking_is_capped_at_the_esc_brake():
    p = load_vehicle()
    assert p.gear_ratio * p.kt * p.esc_regen_current_limit <= p.brake_torque_max
    assert p.esc_regen_current_limit < p.esc_current_limit


def test_variants():
    assert variant_names()[0] == 'nominal'
    assert load_vehicle(variant='heavy').mass == 3.6
    assert load_vehicle(variant='low_mu').mu == 0.75
    with pytest.raises(ValueError, match='unknown variant'):
        load_vehicle(variant='moon')


def test_refuses_an_incomplete_or_untagged_file(tmp_path):
    with open(DEFAULT_FILE) as f:
        data = yaml.safe_load(f)
    del data['vehicle']['mass']
    path = tmp_path / 'v.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='missing'):
        load_vehicle(path)
    data['vehicle']['mass'] = {'value': 3.5, 'source': 'hunch'}
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='source'):
        load_vehicle(path)


def test_lf_must_lie_inside_the_wheelbase():
    with pytest.raises(ValueError, match='lf'):
        load_vehicle(lf=0.4)
