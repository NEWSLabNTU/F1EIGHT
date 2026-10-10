"""The speed step test: a schedule of target speeds, each held for a while.

ROS-free. speed_step_planner_node turns the schedule into /planning/trajectory
through the pursuit shell's own clamps and velocity profile, so the step test
reaches the motor along exactly the path the pursuit lab does: trajectory ->
Autoware control -> vehicle_cmd_gate -> actuator. Nothing downstream changes,
so the operation mode, the gate, MRM and the actuator's watchdog and stop modes
all stay live while it runs.
"""

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple, Union

DEFAULT_SPEEDS = (0.0, 0.5, 1.0, 0.5, 1.5, 0.0)
DEFAULT_HOLD = 4.0


@dataclass
class StepSchedule:
    """Target speeds [m/s] and how long each is held [s]."""

    speeds: List[float] = field(default_factory=lambda: list(DEFAULT_SPEEDS))
    holds: List[float] = field(
        default_factory=lambda: [DEFAULT_HOLD] * len(DEFAULT_SPEEDS))

    def __post_init__(self):
        self.speeds = [float(v) for v in self.speeds]
        self.holds = [float(h) for h in self.holds]
        if not self.speeds:
            raise ValueError('a step schedule needs at least one speed')
        if len(self.holds) != len(self.speeds):
            raise ValueError(
                f'{len(self.speeds)} speeds but {len(self.holds)} hold times')
        for v in self.speeds:
            if not math.isfinite(v) or v < 0.0:
                raise ValueError(f'step speeds must be finite and >= 0, got {v}')
        for h in self.holds:
            if not math.isfinite(h) or h <= 0.0:
                raise ValueError(f'hold times must be finite and > 0, got {h}')

    @classmethod
    def build(cls, speeds: Sequence[float],
              hold: Union[float, Sequence[float]] = DEFAULT_HOLD) -> 'StepSchedule':
        """One hold for every step, or one hold per step."""
        speeds = list(speeds)
        if isinstance(hold, (int, float)):
            holds = [float(hold)] * len(speeds)
        else:
            holds = list(hold)
            if len(holds) == 1:
                holds = holds * len(speeds)
        return cls(speeds, holds)

    @property
    def duration(self) -> float:
        return sum(self.holds)

    def target(self, t: float) -> Tuple[float, int]:
        """(target speed, step index) at `t` seconds into the schedule.

        Before the start the target is 0 and the index -1; after the end it is
        0 and the index len(speeds): the schedule always ends stopped, whatever
        its last entry says.
        """
        if t < 0.0:
            return 0.0, -1
        end = 0.0
        for i, (v, h) in enumerate(zip(self.speeds, self.holds)):
            end += h
            if t < end:
                return v, i
        return 0.0, len(self.speeds)

    def distance(self) -> float:
        """Distance the schedule covers if every step were reached at once [m];
        a lower bound on the space it needs is this plus the braking."""
        return sum(v * h for v, h in zip(self.speeds, self.holds))

    def describe(self) -> str:
        steps = ', '.join(f'{v:.2f} m/s x {h:.1f} s'
                          for v, h in zip(self.speeds, self.holds))
        return (f'{steps}; {self.duration:.1f} s, at least '
                f'{self.distance():.1f} m of straight track')
