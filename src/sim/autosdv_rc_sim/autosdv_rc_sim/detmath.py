"""Transcendental functions from IEEE-754 basic arithmetic only.

Grading must be bit-identical on any machine. CPython's + - * / and
math.sqrt are correctly rounded, so they are. math.sin, math.atan, math.exp and
float ** call the platform libm, which glibc dispatches per CPU (FMA and
non-FMA variants), so their last bit can differ between two laptops. Every
module on the lockstep path takes its trigonometry from here instead;
test_determinism.py refuses libm calls in those modules.

Range reduction is Cody-Waite with fdlibm's split constants; the series have
fixed term counts, so the result never depends on a convergence test. The
accuracy is a few ulp over the ranges the model uses, far below any physical
tolerance. Determinism is the point.
"""
import math

PI = math.pi
HALF_PI = 0.5 * math.pi
sqrt = math.sqrt

_TWO_PI_HI = 6.28318530717958623200e+00
_TWO_PI_LO = 2.44929359829470635445e-16
_HALF_PI_HI = 1.57079632679489655800e+00
_HALF_PI_LO = 6.12323399573676603587e-17
_INV_TWO_PI = 1.0 / _TWO_PI_HI
_INV_HALF_PI = 1.0 / _HALF_PI_HI
_LN2_HI = 6.93147180369123816490e-01
_LN2_LO = 1.90821492927058770002e-10
_INV_LN2 = 1.44269504088896338700e+00
_TAN_PI_12 = 0.26794919243112270
_SQRT3 = 1.7320508075688772
_PI_6 = PI / 6.0


def _sin_poly(r):
    """sin on |r| <= pi/4."""
    r2 = r * r
    term = r
    total = r
    for n in range(1, 12):
        term = term * (-r2) / ((2 * n) * (2 * n + 1))
        total += term
    return total


def _cos_poly(r):
    """cos on |r| <= pi/4."""
    r2 = r * r
    term = 1.0
    total = 1.0
    for n in range(1, 12):
        term = term * (-r2) / ((2 * n - 1) * (2 * n))
        total += term
    return total


def _reduce(x):
    """x = 2 pi k + q pi/2 + r with |r| <= pi/4; return (r, q mod 4)."""
    k = round(x * _INV_TWO_PI)
    r = (x - k * _TWO_PI_HI) - k * _TWO_PI_LO
    q = round(r * _INV_HALF_PI)
    r = (r - q * _HALF_PI_HI) - q * _HALF_PI_LO
    return r, q & 3


def sin(x):
    r, q = _reduce(x)
    if q == 0:
        return _sin_poly(r)
    if q == 1:
        return _cos_poly(r)
    if q == 2:
        return -_sin_poly(r)
    return -_cos_poly(r)


def cos(x):
    r, q = _reduce(x)
    if q == 0:
        return _cos_poly(r)
    if q == 1:
        return -_sin_poly(r)
    if q == 2:
        return -_cos_poly(r)
    return _sin_poly(r)


def atan(x):
    if x != x:
        return x
    negative = x < 0.0
    if negative:
        x = -x
    invert = x > 1.0
    if invert:
        x = 1.0 / x
    offset = 0.0
    if x > _TAN_PI_12:
        # atan(x) = pi/6 + atan((x sqrt3 - 1) / (x + sqrt3)), |argument| <= tan(pi/12)
        x = (x * _SQRT3 - 1.0) / (x + _SQRT3)
        offset = _PI_6
    x2 = x * x
    term = x
    total = x
    for k in range(1, 24):
        term = term * (-x2)
        total += term / (2 * k + 1)
    result = offset + total
    if invert:
        result = HALF_PI - result
    return -result if negative else result


def atan2(y, x):
    if x > 0.0:
        return atan(y / x)
    if x < 0.0:
        return atan(y / x) + (PI if y >= 0.0 else -PI)
    if y > 0.0:
        return HALF_PI
    if y < 0.0:
        return -HALF_PI
    return 0.0


def exp(x):
    if x > 709.0:
        return math.inf
    if x < -745.0:
        return 0.0
    k = round(x * _INV_LN2)
    r = (x - k * _LN2_HI) - k * _LN2_LO
    term = 1.0
    total = 1.0
    for n in range(1, 20):
        term = term * r / n
        total += term
    return math.ldexp(total, k)


def tanh(x):
    ax = -x if x < 0.0 else x
    if ax > 20.0:
        t = 1.0
    elif ax < 1e-5:
        t = ax - ax * ax * ax / 3.0
    else:
        t = 1.0 - 2.0 / (exp(2.0 * ax) + 1.0)
    return -t if x < 0.0 else t
