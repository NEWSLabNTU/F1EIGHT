"""Lab 2 Part 1 speed controller: target speed in, motor PWM count out. You complete this file.

The actuator node calls update() 100 times a second while it is in active control,
with the speed the planner wants and the speed the wheel sensor measures, and
sends the PWM count you return to the motor's electronic speed controller (ESC).
The actuator's safety shell sits around your class and always wins:

  - it clamps your output to [min_pwm, max_pwm] (360 .. 470 on the lab car);
  - when the target is about 0 it brakes (brake_pwm) and then holds neutral
    (init_pwm) itself, so update() is not called to finish a stop;
  - on an emergency stop, a command timeout or a stale speed reading it brakes
    and then holds neutral, and it calls reset() before it hands control back;
  - if update() raises or returns something that is not a finite number, it
    puts the motor in neutral; after 3 faults it disables your class.

The PWM scale and what the ESC does with it (the simulated car's numbers; a lab
car's neutral is 380 to 390, its still band about 5 counts either side, and
params['init_pwm'] is always the car's own):

      min_pwm         init_pwm                                          max_pwm
        360 .. brake .. 365 .. still .. 370 .. still .. 375 .. throttle ... 470
        full brake                    neutral                       full throttle

Above neutral the ESC drives the motor. With a duty below what the motor's speed
already generates, it brakes the car lightly by regeneration, which together with
rolling resistance and drag slows the car by about 1.5 to 2 m/s^2: this is all a
controller that never goes below neutral can get. Counts below neutral are the
ESC's brake, stronger (about 3 m/s^2 at 360), but it has a dead band: the first
brake_deadband counts below neutral (365 .. 369) do nothing. So the controller has
two modes:

  - DRIVING: the feed-forward plus the PID, from neutral up to max_pwm_offset;
  - BRAKING: entered when the car is faster than the target by more than
    brake_enter m/s and left when the excess falls below brake_exit m/s (the gap
    between the two is hysteresis: it keeps the mode from chattering). The
    feed-forward and the integral are set aside, and a request proportional to the
    excess speed (gain kp_brake, damped by D) goes to the brake band through
    brake_pwm(), which skips the dead band.

`params` holds the actuator's motor parameters (init_pwm, min_pwm, max_pwm,
brake_pwm, rate, ...) merged with the mapping in config/speed_pid.yaml, which
wins on a collision. Every key is read below with a default.

`measured_speed` is the hall-sensor speed after the shell's low-pass filter; it
is never negative, since the sensor has no direction. `dt` is the time since the
previous call [s], 0.0 on the first call after a start or a reset(): never
divide by it unchecked, and never assume it is 0.01.

Rules: no ROS imports, no file or network access, no sleeping. Import only math,
cmath, dataclasses, typing, collections, enum, functools, itertools, numbers,
statistics, abc, __future__, numpy, and your own modules beside this file.
`python3 selfcheck.py` checks the rules and each TODO.

What is given and what you write:

    given   reading the parameters; the controller's state (reset); the
            feed-forward (feed_forward); the derivative's low-pass filter
            (filter_derivative); the brake band mapping (brake_pwm); the output
            clamp (clamp_pwm); debug_terms
    TODO    the error and its clamp; P; I with the integral clamp, the integration band and conditional integration;
            D on the measurement, through filter_derivative; the braking mode
            (entry and exit with hysteresis, the brake request); the driving
            command with its throttle limit
"""


class SpeedPID:
    """Feed-forward plus PID above neutral, and a braking mode that uses the ESC's brake band."""

    def __init__(self, params: dict):
        get = params.get
        # The motor's PWM scale (the actuator's parameters)
        self.init_pwm = int(get('init_pwm', 370))   # neutral [counts]
        self.min_pwm = int(get('min_pwm', 360))     # lowest count the shell passes
        self.max_pwm = int(get('max_pwm', 470))     # highest count the shell passes
        # The PID (config/speed_pid.yaml). The defaults are a starting point, not tuned.
        self.kp = float(get('kp', 12.0))            # [counts per m/s]
        self.ki = float(get('ki', 10.0))            # [counts per m]
        self.kd = float(get('kd', 0.0))             # [counts per m/s^2]
        self.integral_limit = float(get('integral_limit', 5.0))   # |integral| <= this [counts]
        self.error_limit = float(get('error_limit', 5.0))   # |error| P and I see, driving [m/s]
        self.integral_band = float(get('integral_band', 20.0))  # integrate only when |e| <= this [m/s]
        self.enable_conditional_integration = bool(get('enable_conditional_integration', True))
        self.derivative_filter_alpha = float(get('derivative_filter_alpha', 0.15))  # (0, 1]
        # Authority on either side of neutral [counts], never beyond the PWM scale
        self.max_pwm_offset = min(int(get('max_pwm_offset', 100)), self.max_pwm - self.init_pwm)
        self.max_brake_offset = min(int(get('max_brake_offset', 10)), self.init_pwm - self.min_pwm)
        # The braking mode
        self.kp_brake = float(get('kp_brake', 6.0))        # brake request per m/s too fast
        self.brake_enter = float(get('brake_enter', 0.15))  # enter when e < -brake_enter [m/s]
        self.brake_exit = float(get('brake_exit', 0.05))    # leave when e > -brake_exit [m/s]
        self.brake_deadband = int(get('brake_deadband', 5))  # counts below neutral that do nothing
        # The feed-forward, calibrated on a fresh battery (config/speed_pid.yaml says more)
        self.ff_deadband = float(get('ff_deadband', 5.27))   # d [counts]
        self.ff_gain = float(get('ff_gain', 0.1433))         # g [m/s per count]
        self.reset()

    # ------------------------------------------------------------------ given

    def feed_forward(self, target_speed: float) -> float:
        """Counts above neutral that hold target_speed on level ground (the motor's model).

            PWM = init_pwm + ff_deadband + target_speed / ff_gain     for target_speed > 0
        """
        if target_speed <= 0.0 or self.ff_gain <= 0.0:
            return 0.0
        return self.ff_deadband + target_speed / self.ff_gain

    def filter_derivative(self, raw: float) -> float:
        """Low-pass filter (EMA) for the derivative term: feed it the raw term, use the result.

        The hall sensor reports a new speed only every 50 ms, so the raw derivative
        is a train of spikes; the filter smooths it. reset() sets the filter to 0.
        """
        a = self.derivative_filter_alpha
        self.derivative = a * raw + (1.0 - a) * self.derivative
        return self.derivative

    def brake_pwm(self, request: float) -> int:
        """The PWM for a brake request of `request` counts (> 0 brakes harder), dead band skipped.

            request <= 0:  init_pwm (neutral: coast)
            request  > 0:  init_pwm - min(brake_deadband + request, max_brake_offset)

        so the first count of request is the ESC's first count of brake, and
        max_brake_offset (never past min_pwm) is full brake. max_brake_offset at or
        below brake_deadband disables the brake: the car coasts.
        """
        if request <= 0.0 or self.max_brake_offset <= self.brake_deadband:
            return self.init_pwm
        offset = min(self.brake_deadband + request, float(self.max_brake_offset))
        return self.clamp_pwm(self.init_pwm - offset)

    def clamp_pwm(self, pwm: float) -> int:
        """Round to a whole count inside [min_pwm, max_pwm]."""
        return int(round(max(self.min_pwm, min(self.max_pwm, pwm))))

    def debug_terms(self):
        """(P, I, D) of the last update; the actuator publishes them on ~/debug/pid_values."""
        return (self.proportional, self.integral, self.derivative)

    def reset(self) -> None:
        """Forget all internal state; the next update() must behave as on a fresh object."""
        self.proportional = 0.0      # the P term of the last update [counts]
        self.integral = 0.0          # the I term [counts]
        self.derivative = 0.0        # the filtered D term [counts] (filter_derivative's state)
        self.last_measured = None    # the previous measured_speed, for D on the measurement
        self.saturated = False       # did the previous driving output hit a limit (anti-windup)?
        self.braking = False         # in the braking mode?

    # ------------------------------------------------------------------- yours

    def update(self, target_speed: float, measured_speed: float, dt: float) -> int:
        """Return the motor PWM count for this tick; init_pwm is neutral."""
        # TODO 1: the error, e = target_speed - measured_speed, and the driving error
        #   e_d: e clamped to [-error_limit, +error_limit]. A large error is mostly the
        #   feed-forward's job (a new target) or a sensor artifact (the hall sensor
        #   reads 0 until the car is at about 0.4 m/s), and feedback should not
        #   respond to it at full strength.
        #
        # TODO 2: the proportional term, P = kp * e_d. Keep it in self.proportional, so
        #   debug_terms() can report it.
        #
        # TODO 3: the derivative term D, on the MEASUREMENT, not on the error (a step in
        #   the target must not kick the output):
        #       raw = -kd * (measured_speed - last_measured) / dt
        #   only when dt > 0 and there is a previous measurement: pass raw through
        #   self.filter_derivative() and use what it returns as D. Otherwise (dt == 0, or
        #   no previous measurement, as on the first call after a start or a reset()) raw
        #   is not defined: D is 0 for this call, and filter_derivative() is not called.
        #   Either way, remember measured_speed in self.last_measured for the next call.
        #
        # TODO 4: the BRAKING MODE (self.braking), with hysteresis:
        #   - not braking: enter it when e < -brake_enter (the car too fast by more
        #     than brake_enter);
        #   - braking: leave it when e > -brake_exit.
        #   While braking, drop the feed-forward and the integral (leave self.integral
        #   as it is: it holds what driving needs once the car is slow enough), ask the
        #   brake for a deceleration proportional to the excess speed (the full e,
        #   not e_d), damped by D,
        #       request = -(kp_brake * e + D)
        #   and return self.brake_pwm(request). (kp_brake has its own value because the
        #   brake band is only a few counts wide: a gain that suits driving brakes too
        #   gently.)
        #
        # TODO 5: driving (not braking). The integral term I (self.integral), with
        #   anti-windup:
        #   - integrate only when dt > 0, when the car is near the target
        #     (|e| <= integral_band: a start or a step is the feed-forward's job), and,
        #     if enable_conditional_integration is set, only when the PREVIOUS output
        #     was not saturated (self.saturated):
        #         integral += ki * e_d * dt
        #   - then clamp it to [-integral_limit, +integral_limit], always.
        #   The command, in counts above neutral:
        #       u = feed_forward(target_speed) + P + I + D
        #   limited to [0, max_pwm_offset] (driving never goes below neutral; that is
        #   the braking mode's job). Set self.saturated to True when u was limited,
        #   False otherwise. Return clamp_pwm(init_pwm + the limited u).
        #
        # Until you complete the TODOs the car holds neutral and never moves.
        return self.init_pwm
