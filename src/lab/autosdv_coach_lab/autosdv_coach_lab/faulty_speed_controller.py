r"""A speed controller that is broken on purpose, for the actuator's fault check.

Field protocol section 3.1 check 4, and `scripts/testing/lab2/bench_checks.py`
on the vehicle bench, load it into the actuator:

    just lab2 bench \
        speed_controller_class:=autosdv_coach_lab.faulty_speed_controller:RaisingSpeedPID

`update()` raises every time it is called. The actuator's shell must put the
motor in neutral, retry after `controller_fault_holdoff` (1 s), disable the
plug-in after `controller_max_faults` (3), and keep running.
"""


class RaisingSpeedPID:
    def __init__(self, params: dict):
        self.init_pwm = int(params.get('init_pwm', 370))

    def reset(self) -> None:
        pass

    def update(self, target_speed: float, measured_speed: float, dt: float) -> int:
        raise RuntimeError('RaisingSpeedPID: deliberately broken (bench fault check)')
