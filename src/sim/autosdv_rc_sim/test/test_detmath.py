"""detmath agrees with libm to a few ulp; its value comes from basic arithmetic alone."""
import math

import pytest

from autosdv_rc_sim import detmath as dm


def grid(lo, hi, n):
    return [lo + (hi - lo) * i / (n - 1) for i in range(n)]


@pytest.mark.parametrize('f, ref, xs', [
    (dm.sin, math.sin, grid(-20.0, 20.0, 4001)),
    (dm.cos, math.cos, grid(-20.0, 20.0, 4001)),
    (dm.atan, math.atan, grid(-50.0, 50.0, 4001) + [1e-300, 1e300, -1e300]),
    (dm.exp, math.exp, grid(-30.0, 30.0, 4001)),
    (dm.tanh, math.tanh, grid(-25.0, 25.0, 4001) + [1e-8, -3e-6]),
])
def test_close_to_libm(f, ref, xs):
    for x in xs:
        assert f(x) == pytest.approx(ref(x), rel=4e-15, abs=4e-16), x


def test_atan2_quadrants():
    for y in (-2.0, -0.5, 0.0, 0.5, 2.0):
        for x in (-3.0, -1.0, 1.0, 3.0):
            assert dm.atan2(y, x) == pytest.approx(math.atan2(y, x), rel=4e-15, abs=4e-16)
    assert dm.atan2(1.0, 0.0) == dm.HALF_PI
    assert dm.atan2(-1.0, 0.0) == -dm.HALF_PI
    assert dm.atan2(0.0, 0.0) == 0.0


def test_special_values():
    assert dm.exp(800.0) == math.inf
    assert dm.exp(-800.0) == 0.0
    assert dm.tanh(30.0) == 1.0 and dm.tanh(-30.0) == -1.0
    assert math.isnan(dm.atan(float('nan')))


@pytest.mark.parametrize('x0', [1e3, 4e3, 1e4, 1e5, 1e6])
def test_sin_cos_large_arguments(x0):
    for i in range(500):
        x = x0 + i * 0.37319
        for sgn in (1.0, -1.0):
            assert abs(dm.sin(sgn * x) - math.sin(sgn * x)) <= 1e-14, x
            assert abs(dm.cos(sgn * x) - math.cos(sgn * x)) <= 1e-14, x
