#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from Adafruit_PCA9685 import PCA9685
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_perception_msgs.msg import TrafficSignalArray
from autoware_perception_msgs.msg import TrafficSignalElement
from autoware_perception_msgs.msg import TrafficSignal
from simple_pid import PID
from geometry_msgs.msg import TwistStamped

# Initialize the PCA9685 using the default address (0x40).
pwm = PCA9685(address=0x40, busnum=7)

# Set the PWM frequency to control the servo and ESC.
pwm.set_pwm_freq(60)

# This section sets the default values for the ESC
fwdmax = 395
revmax = 180
stop = 380  # No throttle value accepted by the ESC

# This section sets the default values for the steering servo
# steering_value = 360
steering_init = 360
steering_max_left = 240
steering_max_right = 480

# Vehicle information
wheel_base = 0.319

# Initialize the PID controllers
speed_pid = PID(Kp=3.0, Ki=0.0, Kd=0.0, output_limits=(revmax-stop, fwdmax-stop))
steering_pid = PID(Kp=100, Ki=0.0, Kd=0.0, output_limits=(steering_max_left-steering_init, steering_max_right-steering_init))


class Esc_control(Node):
    def __init__(self):
        super().__init__("esc_control")

        # subscribe control command from autoware
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            #'/external/selected/control_cmd',
            "/control/command/control_cmd",
            self.control_callback,
            10
        )

        # subscribe to IMU data
        self.imu_subscription = self.create_subscription(
            TwistStamped, "/filter/twist", self.imu_callback, 10
        )


        # publish vehicle status to autoware
        self.pub1 = self.create_publisher(
            SteeringReport, "vehicle/status/steering_status", 10
        )
        self.pub2 = self.create_publisher(
            VelocityReport, "vehicle/status/velocity_status", 10
        )

        # publish traffic signal to autoware
        self.tl_pub = self.create_publisher(
            TrafficSignalArray,
            "perception/traffic_light_recognition/traffic_signals",
            10,
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.target_speed = 0.0
        self.current_speed = 0.0
        self.pwm_offset = 0.0
        self.pwm_value = stop
        self.rev_flag = 0

        self.target_tire_angle = 0.0
        self.current_tire_angle = 0.0
        self.current_steering_accel = 0.0
        self.steering_offset = 0.0
        self.steering_value = steering_init


    def imu_callback(self, msg):
        self.current_speed = msg.twist.linear.x
        self.current_steering_accel = msg.twist.angular.z


    def control_callback(self, msg):
        self.target_speed = msg.longitudinal.speed
        self.target_tire_angle = -msg.lateral.steering_tire_angle


        
    def timer_callback(self):

        # Calculate speed pid
        speed_pid.setpoint = self.target_speed
        self.pwm_offset = float(speed_pid(self.current_speed))

        if abs(self.pwm_offset) < 5:
            self.pwm_value = stop
        else:
            self.pwm_value = int(self.pwm_offset) + stop

        ## Reverse protection
        if self.pwm_value < stop and self.rev_flag >= -40:
            self.pwm_value = stop + int(self.pwm_offset)
            self.rev_flag -= 1
        elif self.pwm_value < stop and self.rev_flag == -41:
            self.pwm_value = stop
            self.rev_flag = -42
        elif self.pwm_value >= stop:
            self.rev_flag = 0

        # Convert angular velovity to steering angle
        self.current_tire_angle = math.atan(self.current_steering_accel * wheel_base / self.current_speed)

        # Calculate steering pid
        steering_pid.setpoint = self.target_tire_angle
        self.steering_offset = float(steering_pid(self.current_tire_angle))

        self.steering_value = int(self.steering_offset) + steering_init

        # Set the PCA9685 servo controller (dc motor and steering servo)
        if revmax < self.pwm_value < fwdmax:
            pwm.set_pwm(0, 0, self.pwm_value)
            self.get_logger().info(
                f"speed value: {self.pwm_value} target: {self.target_speed} current: {self.current_speed}"
            )

        if steering_max_left < self.steering_value < steering_max_right:
            pwm.set_pwm(1, 0, self.steering_value)
            self.get_logger().info(
                f"steering_value: {self.steering_value} target: {self.target_tire_angle} current: {self.current_tire_angle}"
            )

        steering_report = SteeringReport()
        steering_report.steering_tire_angle = self.current_tire_angle
        self.pub1.publish(steering_report)

        velocity_report = VelocityReport()
        velocity_report.longitudinal_velocity = self.current_speed
        velocity_report.header.stamp = self.get_clock().now().to_msg()
        self.pub2.publish(velocity_report)

        # tse = TrafficSignalElement()
        # tse.color = 3  # GREEN
        # tse.shape = 1  # CIRCLE
        # tse.status = 2  # SOLID_ON
        # tse.confidence = 1.0
        # ts = TrafficSignal()
        # ts.traffic_signal_id = 1
        # ts.elements.append(tse)
        # tsa = TrafficSignalArray()
        # tsa.stamp = self.get_clock().now().to_msg()
        # tsa.signals.append(ts)
        # self.tl_pub.publish(tsa)


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


if __name__ == "__main__":
    main()
