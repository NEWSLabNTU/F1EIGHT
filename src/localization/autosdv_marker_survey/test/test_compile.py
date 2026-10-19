"""Every module in the package byte-compiles, including the ROS node that no
other test imports (a syntax error there once shipped unnoticed)."""

import pathlib
import py_compile

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1] / 'autosdv_marker_survey'


@pytest.mark.parametrize('path', sorted(PKG.glob('*.py')), ids=lambda p: p.name)
def test_compiles(path):
    py_compile.compile(str(path), doraise=True)
