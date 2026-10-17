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
