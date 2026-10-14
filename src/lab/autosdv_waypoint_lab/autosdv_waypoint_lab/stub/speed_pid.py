"""Student starting point: a SpeedPID that leaves the motor in neutral.

The student branch's launch files load this class until a team passes its own
with speed_controller_class:=. The real starting point, with the notes on
what the actuator's safety shell already does for you, is the handout's
lab2_waypoint/speed_pid.py; this one only keeps the car still, so a stack
launched with no SpeedPID does not drive.

    update(target_speed, measured_speed, dt) -> motor PWM count

init_pwm is neutral. The actuator clamps the result to [min_pwm, max_pwm],
overrides it on a timeout, an emergency or an MRM, and puts the motor in
neutral if update() raises or returns a non-finite number.
"""


class SpeedPID:
    def __init__(self, params: dict):
        self.init_pwm = int(params.get('init_pwm', 370))
        self.reset()

    def reset(self) -> None:
        pass

    def update(self, target_speed: float, measured_speed: float, dt: float) -> int:
        return self.init_pwm
