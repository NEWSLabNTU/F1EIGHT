"""Reference PathTracker: adaptive-lookahead pure pursuit with a curvature speed limit.

Pure pursuit about the rear axle (base_link): pick the path point a lookahead
distance L ahead of the car's projection, and steer the arc through it,

    alpha = heading of the target in the car's frame
    delta = atan(2 * wheel_base * sin(alpha) / L)

with L = clamp(k_lookahead * v + lookahead_min, lookahead_min, lookahead_max),
so the car looks further ahead (and corners more smoothly) when fast.

Speed: the waypoint speed, limited on curves to sqrt(a_lat_max / |kappa|),
where kappa is the largest path curvature within `preview` metres ahead, so
the car is already slow when it enters the turn.
"""

import math

from autosdv_waypoint_lab.tracker_api import TrackerCommand


class PathTracker:
    def __init__(self, params: dict, vehicle):
        self.wheel_base = vehicle.wheel_base
        self.max_steer = vehicle.max_steer_angle
        self.k_lookahead = float(params.get('k_lookahead', 0.6))
        self.lookahead_min = float(params.get('lookahead_min', 0.6))
        self.lookahead_max = float(params.get('lookahead_max', 2.0))
        self.a_lat_max = float(params.get('a_lat_max', 0.6))
        self.preview = float(params.get('preview', 1.0))

    def reset(self) -> None:
        pass

    def step(self, state, path, dt: float) -> TrackerCommand:
        proj = path.project(state.x, state.y)
        v = max(state.speed, 0.0)
        lookahead = min(max(self.k_lookahead * v + self.lookahead_min, self.lookahead_min),
                        self.lookahead_max)
        tx, ty, _ = path.point_at(proj.s + lookahead)
        dx, dy = tx - state.x, ty - state.y
        alpha = math.atan2(dy, dx) - state.yaw
        dist = max(math.hypot(dx, dy), 1e-3)
        steer = math.atan2(2.0 * self.wheel_base * math.sin(alpha), dist)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        speed = path.speed_at(proj.s)
        k_max = 0.0
        for s_i, k_i in zip(path.s, path.kappa):
            if proj.s - 0.2 <= s_i <= proj.s + self.preview:
                k_max = max(k_max, abs(k_i))
        if k_max > 1e-6:
            speed = min(speed, math.sqrt(self.a_lat_max / k_max))
        return TrackerCommand(steer, speed)
