"""PathTrackers broken on purpose, for the tracker shell's fault check.

    just lab2 bench tracker_class:=autosdv_waypoint_lab.faulty_path_tracker:RaisingPathTracker

`path_tracker` must command speed 0, keep running, and disable the law after
`max_faults` (3) consecutive faults.
"""

from autosdv_waypoint_lab.tracker_api import TrackerCommand


class RaisingPathTracker:
    def __init__(self, params: dict, vehicle):
        pass

    def reset(self) -> None:
        pass

    def step(self, state, path, dt: float) -> TrackerCommand:
        raise RuntimeError('RaisingPathTracker: deliberately broken (bench fault check)')


class NanPathTracker(RaisingPathTracker):
    def step(self, state, path, dt: float) -> TrackerCommand:
        return TrackerCommand(float('nan'), 1.0)
