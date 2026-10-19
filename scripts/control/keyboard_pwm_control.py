#!/usr/bin/env python3
"""
Keyboard PWM control - direct PWM control via keyboard (non-ROS version).
This script allows direct PWM control of motor and steering using keyboard input.

    python3 scripts/control/keyboard_pwm_control.py --vehicle-id car_c
    python3 scripts/control/keyboard_pwm_control.py --vehicle-id car_c --stop 385 --motor-step 2

Every number comes from the car's profile, the file the vehicle interface
loads (individual_params/config/<vehicle_id>/autosdv_vehicle/actuator.param.yaml
over actuator.yaml), so this tool and the stack agree on what "stop" is:

- stop       init_pwm     the motor command that holds the car still
- brake      brake_pwm    the brake pulse of the forward-to-reverse sequence
- motor      [min(brake_pwm, min_pwm), max_pwm]   the clamp for w/s
- centre     init_steer   steering straight ahead
- steering   [min_steer, max_steer]               the clamp for a/d (reversed:
                                                  higher PWM = left)
- I2C        i2c_address, i2c_busnum, pwm_freq

Any of them can be overridden on the command line (--stop, --brake,
--motor-min, --motor-max, --centre, --steer-min, --steer-max) to try a value
before writing it into the profile. The cars differ (car A stops at 370, car
B at 395, car C in a 380-390 band), so there is no built-in default.
"""

import argparse
import os
import sys
import termios
import tty
import select
import time
from Adafruit_PCA9685 import PCA9685

# The car profiles' loader, from the vehicle interface's source tree (ROS not needed).
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src',
                                'vehicle', 'autosdv_vehicle_launch', 'autosdv_vehicle_interface'))
from autosdv_vehicle_interface import car_profile  # noqa: E402


class KeyboardPWMControl:
    """
    Keyboard-based PWM control.

    Controls motor and steering PWM directly using keyboard input.
    """

    def __init__(self, limits, motor_step=5, steering_step=5):
        """Initialize PWM control. ``limits`` comes from pwm_limits()."""
        # Parameters
        self.motor_step = motor_step
        self.steering_step = steering_step
        self.limits = limits

        # Initialize PCA9685 PWM driver
        try:
            self.driver = PCA9685(address=limits['i2c_address'], busnum=limits['i2c_busnum'])
            self.driver.set_pwm_freq(limits['pwm_freq'])
            print('PCA9685 initialized successfully')
        except Exception as e:
            print(f'ERROR: Failed to initialize PCA9685: {e}')
            raise

        # PWM channels
        self.MOTOR_CHANNEL = 0
        self.STEER_CHANNEL = 1

        # Current PWM values, from the car's profile
        self.STOP_PWM = limits['stop']
        self.BRAKE_PWM = limits['brake']
        self.CENTER_STEER = limits['centre']
        self.motor_pwm = self.STOP_PWM
        self.steering_pwm = self.CENTER_STEER

        # PWM limits
        self.MIN_MOTOR_PWM = limits['motor_min']
        self.MAX_MOTOR_PWM = limits['motor_max']
        self.MIN_STEER_PWM = limits['steer_min']
        self.MAX_STEER_PWM = limits['steer_max']

        # Set initial values
        self.set_pwm()

        # Save terminal settings for raw input
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            self.terminal_available = True
        except termios.error:
            print('WARNING: No terminal available - keyboard input disabled')
            self.old_settings = None
            self.terminal_available = False

        self.running = True

    def print_help(self):
        """Print keyboard control help."""
        print('=== Keyboard PWM Control ===')
        print('Motor Control:')
        print('  w/s : Increase/Decrease motor PWM')
        print('  q   : Emergency stop (with brake sequence if moving forward)')
        print('Steering Control:')
        print('  a/d : Right/Left steering (reversed)')
        print(f'  e   : Center steering (PWM = {self.CENTER_STEER})')
        print('Other:')
        print('  h   : Show this help')
        print('  Ctrl-C : Quit')
        print('')
        print(f'Car {self.limits["vehicle_id"]}: motor stop {self.STOP_PWM}, brake {self.BRAKE_PWM}, '
              f'clamp {self.MIN_MOTOR_PWM}..{self.MAX_MOTOR_PWM}')
        print(f'Steering: centre {self.CENTER_STEER}, {self.MAX_STEER_PWM}=left, '
              f'{self.MIN_STEER_PWM}=right')
        print('============================')
        self.log_status()

    def set_pwm(self):
        """Set PWM values to hardware."""
        try:
            self.driver.set_pwm(self.MOTOR_CHANNEL, 0, self.motor_pwm)
            self.driver.set_pwm(self.STEER_CHANNEL, 0, self.steering_pwm)
        except Exception as e:
            print(f'ERROR: Failed to set PWM: {e}')

    def execute_brake_sequence(self):
        """Execute the brake sequence (brake -> stop) that arms backward motion."""
        try:
            print(f'Executing brake sequence: {self.BRAKE_PWM} -> {self.STOP_PWM}')
            self.driver.set_pwm(self.MOTOR_CHANNEL, 0, self.BRAKE_PWM)
            time.sleep(0.2)
            self.driver.set_pwm(self.MOTOR_CHANNEL, 0, self.STOP_PWM)
            time.sleep(0.1)
            print('Brake sequence complete')
        except Exception as e:
            print(f'ERROR: Brake sequence failed: {e}')

    def log_status(self):
        """Log current PWM status."""
        motor_status = "STOP"
        if self.motor_pwm > self.STOP_PWM:
            motor_status = "FORWARD"
        elif self.motor_pwm < self.STOP_PWM:
            motor_status = "REVERSE"

        steer_status = "CENTER"
        if self.steering_pwm < self.CENTER_STEER:
            steer_status = "RIGHT"  # Reversed: lower PWM = right
        elif self.steering_pwm > self.CENTER_STEER:
            steer_status = "LEFT"   # Reversed: higher PWM = left

        print(
            f'Motor: {self.motor_pwm} ({motor_status}), '
            f'Steering: {self.steering_pwm} ({steer_status})'
        )

    def get_key_nonblocking(self):
        """Get a single keypress (non-blocking)."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def process_key(self, key):
        """Process keyboard input."""
        changed = False

        # Motor control
        if key == 'w' or key == 'W':
            self.motor_pwm = min(self.motor_pwm + self.motor_step, self.MAX_MOTOR_PWM)
            changed = True
        elif key == 's' or key == 'S':
            self.motor_pwm = max(self.motor_pwm - self.motor_step, self.MIN_MOTOR_PWM)
            changed = True
        elif key == 'q' or key == 'Q':
            print('EMERGENCY STOP!')
            # If moving forward, execute brake sequence
            if self.motor_pwm > self.STOP_PWM:
                self.execute_brake_sequence()
                self.motor_pwm = self.STOP_PWM
            else:
                # Not moving forward, just stop
                self.motor_pwm = self.STOP_PWM
            changed = True

        # Steering control (reversed: a=right, d=left)
        elif key == 'a' or key == 'A':
            self.steering_pwm = min(self.steering_pwm + self.steering_step, self.MAX_STEER_PWM)
            changed = True
        elif key == 'd' or key == 'D':
            self.steering_pwm = max(self.steering_pwm - self.steering_step, self.MIN_STEER_PWM)
            changed = True
        elif key == 'e' or key == 'E':
            self.steering_pwm = self.CENTER_STEER
            changed = True

        # Other commands
        elif key == 'h' or key == 'H':
            self.print_help()

        if changed:
            self.set_pwm()
            self.log_status()

    def run(self):
        """Main control loop."""
        if not self.terminal_available:
            print('ERROR: No terminal available for keyboard input')
            return

        self.print_help()

        # Set terminal to cbreak mode for immediate keypress detection
        tty.setcbreak(sys.stdin.fileno())

        try:
            while self.running:
                # Non-blocking key read
                key = self.get_key_nonblocking()
                if key:
                    self.process_key(key)
                else:
                    # Small sleep to prevent CPU spinning
                    time.sleep(0.01)
        except KeyboardInterrupt:
            print('\nInterrupted')
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup on shutdown."""
        # Restore terminal settings
        if self.terminal_available and self.old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass

        # Stop motor
        print('\nStopping motor...')
        try:
            self.driver.set_pwm(self.MOTOR_CHANNEL, 0, self.STOP_PWM)
        except:
            pass


def pwm_limits(vehicle_id, overrides):
    """The car's profile numbers this tool uses, with command-line overrides applied."""
    p = car_profile.load(vehicle_id)
    limits = {
        'vehicle_id': vehicle_id,
        'stop': p['init_pwm'], 'brake': p['brake_pwm'],
        'motor_min': min(p['brake_pwm'], p['min_pwm']), 'motor_max': p['max_pwm'],
        'centre': p['init_steer'], 'steer_min': p['min_steer'], 'steer_max': p['max_steer'],
        'i2c_address': p['i2c_address'], 'i2c_busnum': p['i2c_busnum'], 'pwm_freq': p['pwm_freq'],
    }
    limits.update({k: v for k, v in overrides.items() if v is not None})
    if not limits['motor_min'] <= limits['stop'] <= limits['motor_max']:
        raise ValueError(f"stop {limits['stop']} is outside the motor clamp "
                         f"{limits['motor_min']}..{limits['motor_max']}")
    if not limits['steer_min'] <= limits['centre'] <= limits['steer_max']:
        raise ValueError(f"centre {limits['centre']} is outside the steering clamp "
                         f"{limits['steer_min']}..{limits['steer_max']}")
    return limits


def main(argv=None):
    """Main function."""
    ap = argparse.ArgumentParser(description='Drive the motor and steering PWM from the keyboard.')
    car_profile.add_argument(ap)
    ap.add_argument('--motor-step', type=int, default=5, help='w/s step [counts] (5)')
    ap.add_argument('--steering-step', type=int, default=5, help='a/d step [counts] (5)')
    for flag, key, what in (('--stop', 'stop', 'motor stop (init_pwm)'),
                            ('--brake', 'brake', 'brake pulse (brake_pwm)'),
                            ('--motor-min', 'motor_min', 'lowest motor PWM'),
                            ('--motor-max', 'motor_max', 'highest motor PWM (max_pwm)'),
                            ('--centre', 'centre', 'steering centre (init_steer)'),
                            ('--steer-min', 'steer_min', 'lowest steering PWM (min_steer)'),
                            ('--steer-max', 'steer_max', 'highest steering PWM (max_steer)')):
        ap.add_argument(flag, dest=key, type=int, default=None,
                        help=f'override the profile\'s {what}')
    a = ap.parse_args(argv)
    keys = ('stop', 'brake', 'motor_min', 'motor_max', 'centre', 'steer_min', 'steer_max')
    try:
        limits = pwm_limits(a.vehicle_id, {k: getattr(a, k) for k in keys})
    except ValueError as e:
        ap.error(str(e))

    controller = KeyboardPWMControl(limits, motor_step=a.motor_step,
                                    steering_step=a.steering_step)
    controller.run()


if __name__ == '__main__':
    main()
