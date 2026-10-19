import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import new_team as nt  # noqa: E402

LAUNCH = '''<launch>
  <arg name="speed_controller_class" default="autosdv_vehicle_interface.speed_controller:AckermannSpeedPID"
       description="Inner loop"/>
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
STUB_BYTES = b'class SpeedPID:  # TODO\n    pass\n'
LAB = {nt.STUB: STUB_BYTES,
       nt.SIM_SEED: b'# sim\nkp: 12.0\nff_gain: 0.1433\n',
       nt.CAR_SEED: b'# A copy of the Lab 2 handout\'s file\n# Keep equal.\n#\n# car\nkp: 20.0\n'}
IDS = ('car_a', 'sim')


def gen(n, dest, files=FILES, kind='car'):
    return nt.generate(n, dest, files, 'test', LAB, IDS, kind)


@pytest.mark.parametrize('team,model', [(3, 'team3_vehicle'), ('12', 'team12_vehicle')])
def test_model_name(team, model):
    assert nt.model_name(team) == model
    import lab2_ws
    assert lab2_ws.team_of(model) == int(team) and lab2_ws.team_of(lab2_ws.SIM_MODEL) is None


def test_main_writes_the_sim_model_the_template_or_a_team(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(nt, 'read_lab_files', lambda worktree=False: LAB)
    monkeypatch.setattr(nt, 'read_reference', lambda worktree=False: (FILES, 'test'))
    monkeypatch.setattr(nt, 'vehicle_ids', lambda worktree=False: list(IDS))
    assert nt.main(['--sim', '--dest', str(tmp_path)]) == 0
    assert nt.main(['--template', '--dest', str(tmp_path)]) == 0
    assert nt.main(['4', '--dest', str(tmp_path)]) == 0
    assert {p.name for p in tmp_path.iterdir()} == \
        {'lab2_sim_vehicle_launch', 'teamN_vehicle_launch', 'team4_vehicle_launch'}
    assert nt.main(['--template', '--dest', str(tmp_path)]) == 2      # never overwrites
    assert 'exists' in capsys.readouterr().err
    for wrong in (['4', '--sim'], ['--sim', '--template'], []):
        with pytest.raises(SystemExit):
            nt.main(wrong + ['--dest', str(tmp_path)])


@pytest.mark.parametrize('bad', ['0', '-1', 'a', '3a', '1000', ''])
def test_model_name_refuses(bad):
    with pytest.raises(nt.NewTeamError):
        nt.model_name(bad)


def test_generate_renames_packages_and_keeps_the_car_file(tmp_path):
    root = gen(3, tmp_path)
    launch = (root / 'team3_vehicle_launch/launch/vehicle_interface.launch.xml').read_text()
    # the car's PWM points: the fixed individual_params path, untouched
    assert 'config/$(var vehicle_id)/autosdv_vehicle/actuator.param.yaml' in launch
    # the tuning: the team's own file, not the reference's per-car one
    assert '$(find-pkg-share team3_vehicle_launch)/config/$(var vehicle_id)/speed_controller.param.yaml' in launch
    # the model's own SpeedPID, the stub, byte for byte
    assert 'default="team3_vehicle_interface.speed_pid:SpeedPID"' in launch
    assert (root / 'team3_vehicle_interface/team3_vehicle_interface/speed_pid.py').read_bytes() == STUB_BYTES
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
    import yaml
    # the car model: every car, and sim for the vehicle bench, all from the car seed
    assert sorted(p.name for p in (root / 'team3_vehicle_launch/config').iterdir()) == ['car_a', 'sim']
    for vid in ('car_a', 'sim'):
        f = root / f'team3_vehicle_launch/config/{vid}/speed_controller.param.yaml'
        assert yaml.safe_load(f.read_text())['/**']['ros__parameters']['speed_controller_params'] == \
            {'kp': 20.0}
        assert 'Keep equal' not in f.read_text()
    assert 'vehicle bench' in (root / 'team3_vehicle_launch/config/sim/speed_controller.param.yaml').read_text()
    desc = (root / 'team3_vehicle_description/CMakeLists.txt').read_text()
    assert 'find_package(autosdv_vehicle_description REQUIRED)' in desc
    assert not (root / 'team3_vehicle_description/config').exists()   # by reference, no copy
    assert (root / 'README.md').is_file()


def test_the_sim_model_is_one_package_with_the_sim_tuning_only(tmp_path):
    import yaml
    root = gen(3, tmp_path, kind='sim')
    assert root.name == 'lab2_sim_vehicle_launch'                     # no team in its name
    assert [p.name for p in (root / 'lab2_sim_vehicle_launch/config').iterdir()] == ['sim']
    f = root / 'lab2_sim_vehicle_launch/config/sim/speed_controller.param.yaml'
    assert yaml.safe_load(f.read_text())['/**']['ros__parameters']['speed_controller_params'] == \
        {'kp': 12.0, 'ff_gain': 0.1433}
    launch = (root / 'lab2_sim_vehicle_launch/launch/vehicle_interface.launch.xml').read_text()
    assert 'default="lab2_sim_vehicle_interface.speed_pid:SpeedPID"' in launch
    assert (root / 'lab2_sim_vehicle_interface/lab2_sim_vehicle_interface/speed_pid.py').read_bytes() \
        == STUB_BYTES
    # both side by side in one workspace
    gen(3, tmp_path)
    assert (tmp_path / 'team3_vehicle_launch').is_dir()


def test_generate_never_overwrites(tmp_path):
    gen(3, tmp_path)
    with pytest.raises(nt.NewTeamError, match='exists'):
        gen(3, tmp_path)


def test_a_reference_without_the_car_file_is_refused(tmp_path):
    files = dict(FILES)
    files['autosdv_vehicle_launch/launch/vehicle_interface.launch.xml'] = \
        LAUNCH.replace('autosdv_vehicle/actuator.param.yaml', 'x.yaml').encode()
    with pytest.raises(nt.NewTeamError, match='individual_params'):
        gen(3, tmp_path, files)


def test_the_pinned_reference_generates(tmp_path):
    files, source = nt.read_reference()
    root = nt.generate(5, tmp_path, files, source, nt.read_lab_files(worktree=True),
                       nt.vehicle_ids())
    assert {'car_a', 'car_b', 'car_c', 'sim'} == {p.name for p in
                                                 (root / 'team5_vehicle_launch/config').iterdir()}
    assert (root / 'team5_vehicle_launch/launch/vehicle_interface.launch.xml').is_file()
    for p in root.rglob('*'):
        if p.is_file() and p.suffix in ('.py', '.xml', '.cfg'):
            text = p.read_text()
            for ref in ('autosdv_vehicle_interface', 'autosdv_vehicle_launch'):
                assert ref not in text, f'{p}: {ref}'


def test_nothing_ta_only_is_read():
    assert all('reference' not in rel for rel in (nt.STUB, nt.SIM_SEED, nt.CAR_SEED))
    assert all(not rel.startswith(nt.LAB) for rel in nt.read_reference()[0])


# The manual rename the spec describes: every "teamN" in directory and file names, then in
# file contents. RENAME is that recipe as shell, so this test proves the spec's steps work.
RENAME = r"""
set -e
cd "$1"
find . -depth -name '*teamN*' -execdir sh -c 'mv "$1" "$(echo "$1" | sed s/teamN/team3/g)"' _ {} \;
grep -rl teamN . | xargs sed -i s/teamN/team3/g
"""


def test_the_template_renamed_by_hand_builds(tmp_path):
    import shutil
    import subprocess
    files, source = nt.read_reference()
    nt.generate(1, tmp_path / 'src', files, source, nt.read_lab_files(worktree=True),
                nt.vehicle_ids(), 'template')
    subprocess.run(['bash', '-c', RENAME, '_', str(tmp_path / 'src')], check=True)
    left = subprocess.run(['grep', '-rl', 'teamN', str(tmp_path / 'src')], capture_output=True,
                          text=True).stdout
    assert left == ''
    assert not [p for p in (tmp_path / 'src').rglob('*teamN*')]
    root = tmp_path / 'src' / 'team3_vehicle_launch'
    launch = (root / 'team3_vehicle_launch/launch/vehicle_interface.launch.xml').read_text()
    assert 'default="team3_vehicle_interface.speed_pid:SpeedPID"' in launch
    assert (root / 'team3_vehicle_interface/resource/team3_vehicle_interface').is_file()
    if not shutil.which('colcon') or 'autosdv_vehicle_description' not in \
            subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True).stdout:
        pytest.skip('no colcon, or no AutoSDV install sourced: the build is not checked')
    r = subprocess.run(['colcon', 'build', '--base-paths', 'src', '--symlink-install',
                        '--event-handlers', 'console_direct-'], cwd=tmp_path,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    share = tmp_path / 'install/team3_vehicle_launch/share/team3_vehicle_launch'
    assert (share / 'launch/vehicle_interface.launch.xml').is_file()
    assert (share / 'config/car_c/speed_controller.param.yaml').is_file()
    assert (tmp_path / 'install/team3_vehicle_interface/lib/team3_vehicle_interface/actuator').is_file()
