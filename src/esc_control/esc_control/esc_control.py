#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from Adafruit_PCA9685 import PCA9685
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_vehicle_msgs.msg import VelocityReport
from enum import Enum

# from autoware_perception_msgs.msg import TrafficSignalArray
# from autoware_perception_msgs.msg import TrafficSignalElement
# from autoware_perception_msgs.msg import TrafficSignal
from simple_pid import PID
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped

# Initialize the PCA9685 using the default address (0x40).
pwm = PCA9685(address=0x40, busnum=7)

# Set the PWM frequency to control the servo and ESC.
pwm.set_pwm_freq(60)

# This section sets the default values for the ESC
fwdmax = 500
revmax = 260
stop = 380  # No throttle value accepted by the ESC

# This section sets the default values for the steering servo
# steering_value = 360
steering_init = 500
steering_max_left = 380
steering_max_right = 620

# Vehicle information
wheel_base = 0.319

# stop threshold
stop_threshold = 0.1
protection_time_threshold = 0.5

class BrakingState(Enum):
    LEASE = 1
    BRAKING = 2

class Esc_control(Node):
    def __init__(self):
        super().__init__("esc_control")

        # subscribe control command from autoware
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            "/external/selected/control_cmd",
            # "/control/command/control_cmd",
            self.control_callback,
            10,
        )

        # subscribe to IMU data
        self.twist_subscription = self.create_subscription(
            TwistStamped, "/filter/twist", self.twist_callback, 1
        )
        self.accel_subscription = self.create_subscription(
            Vector3Stamped, "/imu/acceleration", self.accel_callback, 1
        )

        # publish vehicle status to autoware
        self.pub1 = self.create_publisher(
            SteeringReport, "vehicle/status/steering_status", 10
        )
        self.pub2 = self.create_publisher(
            VelocityReport, "vehicle/status/velocity_status", 10
        )

        # publish traffic signal to autoware
        # self.tl_pub = self.create_publisher(
        #     TrafficSignalArray,
        #     "perception/traffic_light_recognition/traffic_signals",
        #     10,
        # )

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the PID controllers
        self.speed_pid = PID(Kp=0.18, Ki=0, Kd=0.5, output_limits=(-3, 3))
        self.accel_pid = PID(Kp=10, Ki=0, Kd=0, output_limits=(-120, 120))

        self.target_speed = 0.0
        self.current_speed = 0.0
        self.current_accel = 0.0
        self.pwm_offset = 0.0
        self.pwm_value = stop

        self.target_tire_angle = 0.0
        self.current_tire_angle = 0.0
        self.steering_value = steering_init
        
        self.start_time = None
        self.braking_state = BrakingState.LEASE
        self.rev_flag = None

    def twist_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def accel_callback(self, msg):
        accel = msg.vector.x
        # if abs(accel) <= 1.0:
        #     accel = 0.0
        self.current_accel = accel

    def control_callback(self, msg):
        self.target_speed = msg.longitudinal.speed
        self.target_tire_angle = msg.lateral.steering_tire_angle

    def timer_callback(self):
        # Calculate speed pid
        self.speed_pid.setpoint = self.target_speed
        accel_setpoint = float(self.speed_pid(self.current_speed))
        
        self.accel_pid.setpoint = accel_setpoint
        self.pwm_offset = float(self.accel_pid(self.current_accel))

        print(f"accel_sp = {accel_setpoint}")
        print(f"pwm_off  = {self.pwm_offset}")
        print(f"curr/tar vel = {self.current_speed} / {self.target_speed}")
        print(f"curr/tar acc = {self.current_accel} / {accel_setpoint}")
        print("=========================")

        if abs(self.target_speed) <= stop_threshold:
            self.pwm_value = stop

        if abs(self.current_speed) <= stop_threshold:
            pass
        
        # Braking protection
        if self.pwm_offset < 0:
            if self.rev_flag is None:
                self.pwm_offset = -40
                self.rev_flag = time.time()
            elif time.time() - self.rev_flag <= 0.1:
                self.pwm_offset = -40
            elif time.time() - self.rev_flag <= 0.5:
                self.pwm_offset = 0
            elif self.current_speed >= 0.1:
                if self.braking_state == BrakingState.LEASE:
                    self.start_time = time.time()
                    self.braking_state = BrakingState.BRAKING
                elif time.time() - self.start_time > protection_time_threshold:
                    self.pwm_offset = 0 # protect

        elif self.pwm_offset > 0 and self.current_speed <= -0.1:
            if self.braking_state == BrakingState.LEASE:
                self.start_time = time.time()
                self.braking_state = BrakingState.BRAKING
            elif time.time() - self.start_time > protection_time_threshold:
                self.pwm_offset = 0 # protect
            self.rev_flag = None

        else:
            self.time_slot = None
            self.braking_state = BrakingState.LEASE
            self.rev_flag = None

        self.pwm_value = stop + int(self.pwm_offset)

        # Speed protection
        if self.pwm_value < revmax:
            self.pwm_value = revmax
        elif self.pwm_value > fwdmax:
            self.pwm_value = fwdmax

        
        # There is no sensor to detect tire angle, so we use the target tire angle as the current tire angle.
        self.current_tire_angle = self.target_tire_angle 
        self.steering_value = int(steering_init + 6 * (-self.target_tire_angle) * 180.0/math.pi)

        # Publish reports to Autoware
        steering_report = SteeringReport()
        steering_report.steering_tire_angle = self.current_tire_angle
        self.pub1.publish(steering_report)

        velocity_report = VelocityReport()
        velocity_report.longitudinal_velocity = self.current_speed
        velocity_report.header.stamp = self.get_clock().now().to_msg()
        self.pub2.publish(velocity_report)

        # tse = TrafficSignalElement()
        # tse.color = 3 #GREEN
        # tse.shape = 1 #CIRCLEself.get_logger().info(
        #     f"steering_value: {self.steering_value} target: {self.target_tire_angle} current: {self.current_tire_angle}"
        # )
        # tse.status = 2 #SOLID_ON
        # tse.confidence = 1.0
        # ts = TrafficSignal()
        # ts.traffic_signal_id = 1
        # ts.elements.append(tse)
        # tsa = TrafficSignalArray()
        # tsa.stamp = self.get_clock().now().to_msg()
        # tsa.signals.append(ts)
        # self.tl_pub.publish(tsa)

        # Set the PCA9685 servo controller (dc motor and steering servo)
        pwm.set_pwm(0, 0, self.pwm_value)

        # self.get_logger().info(
        #     f"speed value: {self.pwm_value} target: {self.target_speed} current: {self.current_speed}"
        # )

        pwm.set_pwm(1, 0, self.steering_value)

        # self.get_logger().info(
        #     f"steering_value: {self.steering_value} target: {self.target_tire_angle} current: {self.current_tire_angle}"
        # )

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
