"""Put the source trees first on sys.path, so the tests run on the code in this checkout.

Temporary directories go under ./tmp, not the system /tmp (the repository's rule).
"""
from pathlib import Path
import sys

import pytest

REPO = Path(__file__).resolve().parents[4]
PKG = REPO / 'src/sim/autosdv_rc_sim'
VEHICLE_INTERFACE = REPO / 'src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface'
for path in (str(VEHICLE_INTERFACE), str(PKG)):
    if path in sys.path:
        sys.path.remove(path)
    sys.path.insert(0, path)


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config):
    if config.option.basetemp is None:
        parent = REPO / 'tmp' / 'pytest'
        parent.mkdir(parents=True, exist_ok=True)
        config.option.basetemp = parent / 'autosdv_rc_sim'
