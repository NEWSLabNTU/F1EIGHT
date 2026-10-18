"""The interactive stack's diagnostic graph keeps the 1 Hz /adapi diagnostics from reading STALE.

Those units default to a 1.0 s timeout and are published every 1.0 s; on the plant's
simulated clock the aggregator's tick lands on the same instant and reads them stale once
a second (Task 18b). Their timeout must exceed the publish period.
"""
from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'diagnostics'
PUBLISH_PERIOD = 1.0
UNITS = {
    '/adapi/mrm_request/delegate': 'rc_sim_diagnostics.yaml',
    '/autoware/localization/state': 'localization.yaml',
    '/autoware/planning/routing/state': 'planning.yaml',
}


def _units(name):
    return {u['path']: u for u in yaml.safe_load((CONFIG / name).read_text())['units']}


def test_adapi_diagnostics_outlast_their_publish_period():
    for path, name in UNITS.items():
        unit = _units(name)[path]
        assert unit['type'] == 'diag'
        assert unit.get('timeout', 1.0) >= 2 * PUBLISH_PERIOD, f'{path} in {name}'


def test_graph_includes_the_local_copies_not_upstreams():
    files = [f['path'] for f in yaml.safe_load((CONFIG / 'rc_sim_diagnostics.yaml').read_text())['files']]
    assert '$(dirname)/localization.yaml' in files and '$(dirname)/planning.yaml' in files
    assert not any(f.endswith('/localization.yaml') or f.endswith('/planning.yaml')
                   for f in files if not f.startswith('$(dirname)'))


def _upstream_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('autoware_launch')) / 'config' / 'system' / 'diagnostics'
    except Exception:  # no ROS environment, or autoware_launch not installed
        return None


UPSTREAM_OF = {'rc_sim_diagnostics.yaml': 'autoware-main.yaml',
               'localization.yaml': 'localization.yaml', 'planning.yaml': 'planning.yaml'}


def test_copies_match_upstream_but_for_the_timeouts():
    """After an Autoware upgrade this fails until the copies are re-diffed (and re-copied)."""
    import pytest
    upstream = _upstream_dir()
    if upstream is None or not upstream.is_dir():
        pytest.skip('autoware_launch is not installed')
    for name, up_name in UPSTREAM_OF.items():
        ours = yaml.safe_load((CONFIG / name).read_text())
        theirs = yaml.safe_load((upstream / up_name).read_text())
        for unit in ours['units']:
            if UNITS.get(unit['path']) == name:
                del unit['timeout']
        if 'files' in ours:
            # ours points at upstream's directory for the files it does not copy; compare names
            ours['files'] = sorted(f['path'].rsplit('/', 1)[-1] for f in ours['files'])
            theirs['files'] = sorted(f['path'].rsplit('/', 1)[-1] for f in theirs['files'])
        assert ours == theirs, f'{name} drifted from upstream {upstream / up_name}'
