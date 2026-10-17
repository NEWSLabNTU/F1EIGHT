"""Transcendental functions from IEEE-754 basic arithmetic only.

Grading must be bit-identical on any machine. CPython's + - * / and
math.sqrt are correctly rounded, so they are. math.sin, math.atan, math.exp and
float ** call the platform libm, which glibc dispatches per CPU (FMA and
non-FMA variants), so their last bit can differ between two laptops. Every
module on the lockstep path takes its trigonometry from here instead;
test_determinism.py refuses libm calls in those modules.

Range reduction for sin/cos is Cody-Waite on pi/2, split (fdlibm) into pieces
whose leading parts carry 33 bits, so q * piece is exact for |q| < 2**20,
i.e. |x| <= ~1.6e6. Inside that range the absolute error stays near 1e-16
independent of |x| (checked to 1e-14 up to |x| = 1e6); beyond it the error
grows linearly and is not supported. The series have fixed term counts, so the
result never depends on a convergence test. Determinism is the point.
"""
import math

PI = math.pi
HALF_PI = 0.5 * math.pi
sqrt = math.sqrt

# fdlibm pi/2 split: _PIO2_1 and _PIO2_2 have 33 significant bits, _PIO2_3 too.
_PIO2_1 = 1.57079632673412561417e+00
_PIO2_2 = 6.07710050630396597660e-11
_PIO2_3 = 2.02226624871116645580e-21
_PIO2_3T = 8.47842766036889956997e-32
_INV_HALF_PI = 6.36619772367581382433e-01
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
    """x = q pi/2 + r with |r| <= ~pi/4; return (r, q mod 4)."""
    q = round(x * _INV_HALF_PI)
    r = x - q * _PIO2_1
    r = r - q * _PIO2_2
    r = r - q * _PIO2_3
    r = r - q * _PIO2_3T
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
