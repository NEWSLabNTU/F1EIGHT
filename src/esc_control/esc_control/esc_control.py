#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from Adafruit_PCA9685 import PCA9685

# Initialize the PCA9685 using the default address (0x40).
pwm = PCA9685(address=0x40, busnum=7)

# Set the PWM frequency to control the servo and ESC.
pwm.set_pwm_freq(60)

# This section sets the default values for the ESC
fwdmax = 580
revmax = 180
stop = 380  # No throttle value accepted by the ESC

# This section sets the default values for the steering servo
steering_value = 380
steering_init = 380
steering_max_left = 260
steering_max_right = 500
reverse = 0

class Esc_control(Node):
    def __init__(self):
        super().__init__('esc_control')
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            '/external/selected/control_cmd',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global reverse
        # Access the speed from the longitudinal part of the message
        speed = int(msg.longitudinal.speed) + stop
        if speed < stop and reverse >= -40:
            speed = stop+reverse
            reverse -= 1
        elif speed < stop and reverse == -41:
            speed = stop
            reverse = -42
        elif speed >= stop:
            reverse = 0

        # Access the steering angle from the lateral part of the message
        steering_value = int(-msg.lateral.steering_tire_angle * 380) + steering_init

        # Set the PCA9685 servo controller (dc motor and steering servo)
        if revmax < speed < fwdmax:
            pwm.set_pwm(0, 0, speed)
            print("speed:", speed)

        if steering_max_left < steering_value < steering_max_right:
            pwm.set_pwm(1, 0, steering_value)
            print("steering_value:", steering_value)

def main(args=None):
    rclpy.init(args=args)
    esc_control = Esc_control()
    try:
        rclpy.spin(esc_control)
    except KeyboardInterrupt:
        # Gracefully shutdown on keyboard interrupt
        pass
    finally:
        # Destroy the node explicitly
        esc_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

