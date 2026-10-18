"""The hall sensor: an edge whenever one of the markers on the rear wheel passes.

Edge times are interpolated linearly inside the physics step and rounded to
integer nanoseconds, the unit VelocityEstimator works in. The sensor has no
direction: a marker passing backwards is an edge too.

The sensor keeps its own wheel phase, wrapped into [0, spacing) by subtraction
of whole spacings, and advances it by the step's angle increment only. The
absolute wheel angle never enters an edge computation, so the result does not
degrade after hours of running (an unbounded angle would, in the float
arithmetic and in anything that feeds it to sin/cos).
"""
import math
from typing import List


_BELOW_ONE = 0.9999999999999998   # the largest double under 1: keeps the phase inside [0, spacing)


class HallSensor:
    def __init__(self, markers: int):
        self.spacing = 2.0 * math.pi / markers
        self.start_angle = 0.5 * self.spacing   # never start on a marker
        self.phase = self.start_angle

    def reset(self, theta: float) -> None:
        self.phase = theta - math.floor(theta / self.spacing) * self.spacing

    def step(self, t0_ns: int, dt_ns: int, theta0: float, theta1: float) -> List[int]:
        s = self.spacing
        d = theta1 - theta0
        p0 = self.phase
        p1 = p0 + d
        k = math.floor(p1 / s)
        edges = []
        if k > 0:
            for m in range(1, k + 1):
                edges.append(self._time(t0_ns, dt_ns, p0, d, m * s))
        elif k < 0:
            for m in range(0, k, -1):       # marks 0, -1, ... k+1 are crossed downwards
                edges.append(self._time(t0_ns, dt_ns, p0, d, m * s))
        self.phase = p1 - k * s
        if not 0.0 <= self.phase < s:       # rounding at the boundary
            self.phase = min(max(self.phase, 0.0), s * _BELOW_ONE)
        return edges

    @staticmethod
    def _time(t0_ns: int, dt_ns: int, p0: float, d: float, mark: float) -> int:
        frac = (mark - p0) / d
        return t0_ns + max(1, min(dt_ns, int(round(frac * dt_ns))))
