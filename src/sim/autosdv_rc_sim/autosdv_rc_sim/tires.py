"""Pacejka combined slip at one axle (single track): the similarity method.

    s_x = B_x kappa,  s_y = B_y alpha,  s = |(s_x, s_y)|
    F   = mu Fz sin(C atan(s - E (s - atan s)))
    F_x = F s_x / s,  F_y = F s_y / s

Small-slip stiffness is C B mu Fz; the peak is mu Fz (C >= 1); the combined
force never leaves the friction circle. Wheel frame: x along the wheel,
positive alpha gives a positive (leftward) F_y.
"""
from typing import Tuple

from autosdv_rc_sim.detmath import atan, sin, sqrt


def magic(s: float, c: float, e: float) -> float:
    """Normalised magic-formula shape for a scaled slip s >= 0."""
    return sin(c * atan(s - e * (s - atan(s))))


def combined(kappa: float, alpha: float, fz: float, mu: float,
             bx: float, by: float, c: float, e: float) -> Tuple[float, float]:
    """(F_x, F_y) [N] for longitudinal slip kappa and slip angle alpha [rad] under load fz."""
    if fz <= 0.0:
        return 0.0, 0.0
    sx = bx * kappa
    sy = by * alpha
    s = sqrt(sx * sx + sy * sy)
    if s == 0.0:
        return 0.0, 0.0
    force = mu * fz * magic(s, c, e)
    return force * sx / s, force * sy / s
