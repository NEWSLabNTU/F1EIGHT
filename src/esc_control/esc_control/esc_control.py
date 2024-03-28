#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from Adafruit_PCA9685 import PCA9685
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_perception_msgs.msg import TrafficSignalArray
from autoware_perception_msgs.msg import TrafficSignalElement
from autoware_perception_msgs.msg import TrafficSignal

# Initialize the PCA9685 using the default address (0x40).
pwm = PCA9685(address=0x40, busnum=7)

# Set the PWM frequency to control the servo and ESC.
pwm.set_pwm_freq(60)

# This section sets the default values for the ESC
fwdmax = 580
revmax = 180
stop = 380  # No throttle value accepted by the ESC

# This section sets the default values for the steering servo
steering_value = 360
steering_init = 360
steering_max_left = 240
steering_max_right = 480
reverse = 0

class Esc_control(Node):
    def __init__(self):
        super().__init__('esc_control')
        
        # subscribe control command from autoware
        self.subscription = self.create_subscription(
            AckermannControlCommand,
            #'/external/selected/control_cmd',
            '/control/command/control_cmd',
            self.listener_callback,
            10
        )
	
	
	# publish vehicle status to autoware
        self.pub1 = self.create_publisher(SteeringReport, 'vehicle/status/steering_status', 10)
        self.pub2 = self.create_publisher(VelocityReport, 'vehicle/status/velocity_status', 10)
        
        # publish traffic signal to autoware
        self.tl_pub = self.create_publisher(TrafficSignalArray, 'perception/traffic_light_recognition/traffic_signals', 10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        steering_report = SteeringReport()
        steering_report.steering_tire_angle = 0.0
        self.pub1.publish(steering_report)

        velocity_report = VelocityReport()
        velocity_report.longitudinal_velocity = 0.0
        velocity_report.header.stamp = self.get_clock().now().to_msg()
        self.pub2.publish(velocity_report)
        
        tse = TrafficSignalElement()
        tse.color = 3 #GREEN
        tse.shape = 1 #CIRCLE
        tse.status = 2 #SOLID_ON
        tse.confidence = 1.0
        ts = TrafficSignal()
        ts.traffic_signal_id = 1
        ts.elements.append(tse)
        tsa = TrafficSignalArray()
        tsa.stamp = self.get_clock().now().to_msg()
        tsa.signals.append(ts)
        self.tl_pub.publish(tsa)
        

    def listener_callback(self, msg):
        global reverse
        # Access the speed from the longitudinal part of the message
        speed = int(msg.longitudinal.speed*3) + stop
        if speed < stop and reverse >= -40:
            speed = stop+reverse
            reverse -= 1
        elif speed < stop and reverse == -41:
            speed = stop
            reverse = -42
        elif speed >= stop:
            reverse = 0

        # Access the steering angle from the lateral part of the message
        steering_value = int(-msg.lateral.steering_tire_angle * 550) + steering_init

        # Set the PCA9685 servo controller (dc motor and steering servo)
        if revmax < speed < fwdmax:
            pwm.set_pwm(0, 0, speed)
            self.get_logger().info(f"speed: {speed} autoware value: {msg.longitudinal.speed}")

        if steering_max_left < steering_value < steering_max_right:
            pwm.set_pwm(1, 0, steering_value)
            self.get_logger().info(f"steering_value: {steering_value}  autoware value: {msg.lateral.steering_tire_angle}")
 

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

