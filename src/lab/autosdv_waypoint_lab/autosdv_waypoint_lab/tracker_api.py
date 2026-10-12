"""The student PathTracker's interface, and the shell the node runs it in. No ROS.

A tracker is any class, loaded by import path (`package.module:ClassName`):

    class PathTracker:
        def __init__(self, params: dict, vehicle: VehicleGeometry): ...
        def reset(self) -> None: ...
        def step(self, state: VehicleState, path: Path, dt: float) -> TrackerCommand: ...

`TrackerShell` is what keeps a student bug from driving the car: it clamps the
command to the steering limit and to the path's own speed, turns an exception
or a non-finite output into a stop, and disables the law after
`max_faults` consecutive faults -- the policy the actuator applies to a faulty
SpeedPID.
"""

from dataclasses import dataclass
import importlib
import math
from typing import Optional

from autosdv_waypoint_lab.path import Path


@dataclass(frozen=True)
class VehicleGeometry:
    """vehicle_info.param.yaml's numbers the tracker needs."""

    wheel_base: float = 0.319        # [m] rear axle (base_link) to front axle
    max_steer_angle: float = 0.349   # [rad] tire angle limit


@dataclass(frozen=True)
class VehicleState:
    """The localized car: rear-axle (base_link) pose in the map frame, and speed."""

    x: float
    y: float
    yaw: float
    speed: float


@dataclass
class TrackerCommand:
    """What a tracker asks for: tire angle [rad], left positive; speed [m/s]."""

    steering_angle: float
    speed: float


def load_tracker(spec: str, params: dict, vehicle: VehicleGeometry):
    """Instantiate `package.module:ClassName(params, vehicle)` and check its methods."""
    module_name, sep, class_name = (spec or '').partition(':')
    if not sep or not module_name or not class_name:
        raise ValueError(f"tracker_class must be 'package.module:ClassName', got {spec!r}")
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    obj = cls(dict(params), vehicle)
    for method in ('reset', 'step'):
        if not callable(getattr(obj, method, None)):
            raise ValueError(f'{spec} has no {method}() method')
    return obj


@dataclass
class ShellResult:
    command: TrackerCommand   # what is sent
    raw: Optional[TrackerCommand]   # what the law returned (None on a fault)
    fault: str                # '' or why the law's output was not used
    limit: str                # '' or the clamp that bound: steer, path_speed
    disabled: bool


class TrackerShell:
    def __init__(self, tracker, vehicle: VehicleGeometry, max_faults: int = 3):
        self.tracker = tracker
        self.vehicle = vehicle
        self.max_faults = max_faults
        self.faults = 0
        self.disabled = False
        self.last_steer = 0.0

    def reset(self):
        """Called on engage. A disabled law stays disabled until restart."""
        if self.disabled:
            return
        self.faults = 0
        self.tracker.reset()

    def step(self, state: VehicleState, path: Path, dt: float) -> ShellResult:
        proj = path.project(state.x, state.y)
        v_path = max(0.0, path.speed_at(proj.s))
        if self.disabled:
            return ShellResult(TrackerCommand(self.last_steer, 0.0), None,
                               'disabled', '', True)
        try:
            raw = self.tracker.step(state, path, dt)
            steer = float(raw.steering_angle)
            speed = float(raw.speed)
            if not (math.isfinite(steer) and math.isfinite(speed)):
                raise ValueError(f'non-finite command {raw}')
        except Exception as e:  # noqa: BLE001 -- student code
            self.faults += 1
            if self.faults >= self.max_faults:
                self.disabled = True
            return ShellResult(TrackerCommand(self.last_steer, 0.0), None,
                               f'{type(e).__name__}: {e}', '', self.disabled)
        self.faults = 0
        limit = ''
        lim = self.vehicle.max_steer_angle
        if abs(steer) > lim:
            steer = math.copysign(lim, steer)
            limit = 'steer'
        speed = max(0.0, speed)
        if speed > v_path:
            speed = v_path
            limit = limit + ',path_speed' if limit else 'path_speed'
        self.last_steer = steer
        return ShellResult(TrackerCommand(steer, speed), TrackerCommand(float(raw.steering_angle),
                           float(raw.speed)), '', limit, False)
