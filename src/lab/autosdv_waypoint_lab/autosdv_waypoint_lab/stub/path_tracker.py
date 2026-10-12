"""Student starting point: a PathTracker that drives nowhere.

Replace step() with your law. It receives the localized car (state.x, .y,
.yaw, .speed), the path ahead (path.x, .y, .yaw, .v, .s, .kappa as lists,
plus path.project(x, y), path.point_at(s) and path.speed_at(s)) and the time
since the last call; it returns the tire angle (rad, left positive) and the
speed (m/s) you want. The node clamps the angle to vehicle.max_steer_angle and
the speed to the path's own speed.
"""

from autosdv_waypoint_lab.tracker_api import TrackerCommand


class PathTracker:
    def __init__(self, params: dict, vehicle):
        self.params = params
        self.vehicle = vehicle

    def reset(self) -> None:
        pass

    def step(self, state, path, dt: float) -> TrackerCommand:
        return TrackerCommand(steering_angle=0.0, speed=0.0)
