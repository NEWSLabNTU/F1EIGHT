import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import new_team as nt  # noqa: E402

LAUNCH = '''<launch>
  <let name="car_param_file"
       value="$(find-pkg-share individual_params)/config/$(var vehicle_id)/autosdv_vehicle/actuator.param.yaml"/>
  <!-- the car's tuning of the default speed controller (gains, ESC dead-band
       compensation); empty for a car with no tuning, so actuator.yaml's stand -->
  <let name="car_tuning_file"
       value="$(find-pkg-share individual_params)/config/$(var vehicle_id)/autosdv_vehicle/speed_controller.param.yaml"/>
  <arg name="vehicle_info_param_file" default="$(find-pkg-share autosdv_vehicle_description)/config/vehicle_info.param.yaml"/>
  <node pkg="autosdv_vehicle_interface" exec="actuator" name="actuator_node">
    <param from="$(find-pkg-share autosdv_vehicle_interface)/params/actuator.yaml"/>
    <param from="$(var car_param_file)"/>
    <param from="$(var car_tuning_file)"/>
  </node>
</launch>
'''
FILES = {
    'autosdv_vehicle_launch/launch/vehicle_interface.launch.xml': LAUNCH.encode(),
    'autosdv_vehicle_launch/CMakeLists.txt': b'ament_auto_package(INSTALL_TO_SHARE\n  launch\n)\n',
    'autosdv_vehicle_launch/package.xml': b'<name>autosdv_vehicle_launch</name>'
                                          b'<exec_depend>autosdv_vehicle_interface</exec_depend>',
    'autosdv_vehicle_interface/setup.cfg': b'script_dir=$base/lib/autosdv_vehicle_interface\n',
    'autosdv_vehicle_interface/autosdv_vehicle_interface/actuator.py':
        b'#!/usr/bin/env python3\nfrom autosdv_vehicle_interface.hal import PWM_TOPIC\n',
    'autosdv_vehicle_interface/resource/autosdv_vehicle_interface': b'',
    'autosdv_vehicle_interface/test/golden.bin': b'\x00\xff',
}


@pytest.mark.parametrize('team,model', [(3, 'team3_vehicle'), ('12', 'team12_vehicle')])
def test_model_name(team, model):
    assert nt.model_name(team) == model


@pytest.mark.parametrize('bad', ['0', '-1', 'a', '3a', '1000', ''])
def test_model_name_refuses(bad):
    with pytest.raises(nt.NewTeamError):
        nt.model_name(bad)


def test_generate_renames_packages_and_keeps_the_car_file(tmp_path):
    root = nt.generate(3, tmp_path, FILES, 'test')
    launch = (root / 'team3_vehicle_launch/launch/vehicle_interface.launch.xml').read_text()
    # the car's PWM points: the fixed individual_params path, untouched
    assert 'config/$(var vehicle_id)/autosdv_vehicle/actuator.param.yaml' in launch
    # the tuning: the team's own file, not the reference's per-car one
    assert '$(find-pkg-share team3_vehicle_launch)/config/speed_controller.param.yaml' in launch
    assert 'autosdv_vehicle/speed_controller.param.yaml' not in launch
    assert "the car's tuning of the default speed controller" not in launch
    assert 'pkg="team3_vehicle_interface"' in launch
    assert 'team3_vehicle_description' in launch
    assert 'autosdv_vehicle_interface' not in launch
    act = root / 'team3_vehicle_interface/team3_vehicle_interface/actuator.py'
    assert 'from team3_vehicle_interface.hal' in act.read_text()
    assert act.stat().st_mode & 0o111
    assert (root / 'team3_vehicle_interface/resource/team3_vehicle_interface').is_file()
    assert (root / 'team3_vehicle_interface/test/golden.bin').read_bytes() == b'\x00\xff'
    assert 'config' in (root / 'team3_vehicle_launch/CMakeLists.txt').read_text()
    assert (root / 'team3_vehicle_launch/config/speed_controller.param.yaml').is_file()
    desc = (root / 'team3_vehicle_description/CMakeLists.txt').read_text()
    assert 'find_package(autosdv_vehicle_description REQUIRED)' in desc
    assert not (root / 'team3_vehicle_description/config').exists()   # by reference, no copy
    assert (root / 'README.md').is_file()


def test_generate_never_overwrites(tmp_path):
    nt.generate(3, tmp_path, FILES, 'test')
    with pytest.raises(nt.NewTeamError, match='exists'):
        nt.generate(3, tmp_path, FILES, 'test')


def test_a_reference_without_the_car_file_is_refused(tmp_path):
    files = dict(FILES)
    files['autosdv_vehicle_launch/launch/vehicle_interface.launch.xml'] = \
        LAUNCH.replace('autosdv_vehicle/actuator.param.yaml', 'x.yaml').encode()
    with pytest.raises(nt.NewTeamError, match='individual_params'):
        nt.generate(3, tmp_path, files, 'test')


def test_the_pinned_reference_generates(tmp_path):
    files, source = nt.read_reference()
    root = nt.generate(5, tmp_path, files, source)
    assert (root / 'team5_vehicle_launch/launch/vehicle_interface.launch.xml').is_file()
    for p in root.rglob('*'):
        if p.is_file() and p.suffix in ('.py', '.xml', '.cfg'):
            text = p.read_text()
            for ref in ('autosdv_vehicle_interface', 'autosdv_vehicle_launch'):
                assert ref not in text, f'{p}: {ref}'
