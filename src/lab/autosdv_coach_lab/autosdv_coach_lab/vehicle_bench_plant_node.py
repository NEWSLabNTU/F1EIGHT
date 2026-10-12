"""vehicle_bench_plant: the car's hardware, for the vehicle bench (`just lab2 bench`).

Reads what the actuator commands and publishes what the car's sensors would:

  in   /autosdv/actuator_node/debug/pwm_values   [motor_pwm, steer_pwm], the
       values the actuator writes to the PCA9685 (published every tick, dry
       run included)
  out  /vehicle/status/velocity_status   VelocityReport, 20 Hz, base_link:
       velocity_report's message, from the hall-sensor model
       /sensing/camera/zedxm/imu/data    Imu in zedxm_imu_link: the ZED IMU's
       raw topic, so imu_corrector and dead reckoning run unchanged; it carries
       the static gyro bias imu_corrector.param.yaml removes
       ~/odometry                        the TRUE pose (map, starting at the
       origin like dead reckoning, or at initial_x/y/yaw, or at a waypoint
       course's start box with start_pose_file); virtual_coach places the
       board from it, virtual_marker_localizer the marker sightings

The model is bench_plant.VehiclePlant: the motor fit's MotorPlant, the
actuator's steering mapping inverted, bicycle kinematics, HallSpeedSensor.

When the PWM stream stops (the actuator exited) the motor goes to neutral after
`pwm_timeout`: every exit path of the actuator writes neutral, but a dry run
does not publish that write. `on_pwm_loss: hold` keeps the last PWM instead,
as the PCA9685 does after a SIGKILL.
"""

import random

import rclpy
from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
from autoware_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
import tf2_ros
import yaml

from autosdv_coach_lab.bench_plant import (
    SteeringParams, VehiclePlant, quat_conjugate, quat_from_yaw, quat_multiply, quat_rotate)
from autosdv_vehicle_interface.motor_plant import MotorPlantParams

GRAVITY = 9.80665


class VehicleBenchPlant(Node):
    def __init__(self):
        super().__init__('vehicle_bench_plant')
        d = SteeringParams()
        declare = self.declare_parameter
        # topics and frames
        declare('pwm_topic', '/autosdv/actuator_node/debug/pwm_values')
        declare('velocity_topic', '/vehicle/status/velocity_status')
        declare('imu_topic', '/sensing/camera/zedxm/imu/data')
        declare('imu_frame', 'zedxm_imu_link')
        declare('base_frame', 'base_link')
        declare('odom_frame', 'map')
        # rates: `rate` and `frame_id` come from velocity_report.yaml
        declare('rate', 20.0)
        declare('frame_id', 'base_link')
        declare('physics_rate', 200.0)
        declare('imu_rate', 100.0)
        declare('odom_rate', 50.0)
        # actuator.yaml and vehicle_info.param.yaml, loaded by the launch file
        declare('init_pwm', 370)
        declare('init_steer', d.init_steer)
        declare('min_steer', d.min_steer)
        declare('max_steer', d.max_steer)
        declare('tire_angle_to_steer_ratio', d.tire_angle_to_steer_ratio)
        declare('max_steer_angle', d.max_steer_angle)
        declare('wheel_base', d.wheel_base)
        # velocity_report.yaml (wheel diameter in cm, as there)
        declare('wheel_diameter', 10.5)
        declare('markers_per_rotation', 12)
        # the model
        declare('plant', '')    # MotorPlantParams overrides, a YAML mapping (fit-motor's output)
        declare('steer_time_constant', d.time_constant)
        declare('gyro_noise_std', 0.002)
        # imu_corrector.param.yaml, loaded by the launch file: the static bias
        # imu_corrector subtracts. The raw IMU here carries exactly that bias,
        # so the corrected rate is the true one; gyro_bias_error_z adds a
        # residual on top, for a drift test.
        declare('angular_velocity_offset_x', 0.0)
        declare('angular_velocity_offset_y', 0.0)
        declare('angular_velocity_offset_z', 0.0)
        declare('gyro_bias_error_z', 0.0)
        declare('pwm_timeout', 0.5)
        declare('on_pwm_loss', 'neutral')
        # where the car starts: the map origin (dead reckoning's), or a pose,
        # or the start_pose of a waypoint course.yaml (which wins)
        declare('initial_x', 0.0)
        declare('initial_y', 0.0)
        declare('initial_yaw', 0.0)
        declare('start_pose_file', '')
        gp = lambda k: self.get_parameter(k).value  # noqa: E731

        overrides = yaml.safe_load(gp('plant') or '') or {}
        if not isinstance(overrides, dict):
            raise ValueError(f"plant must be a YAML mapping, got {gp('plant')!r}")
        overrides.setdefault('init_pwm', int(gp('init_pwm')))
        motor = MotorPlantParams(**overrides)
        steering = SteeringParams(
            init_steer=int(gp('init_steer')), min_steer=int(gp('min_steer')),
            max_steer=int(gp('max_steer')),
            tire_angle_to_steer_ratio=float(gp('tire_angle_to_steer_ratio')),
            max_steer_angle=float(gp('max_steer_angle')),
            wheel_base=float(gp('wheel_base')),
            time_constant=float(gp('steer_time_constant')))
        self.plant = VehiclePlant(
            motor, steering, wheel_diameter=float(gp('wheel_diameter')) / 100.0,
            markers_per_rotation=int(gp('markers_per_rotation')))
        x0, y0, yaw0 = float(gp('initial_x')), float(gp('initial_y')), float(gp('initial_yaw'))
        if gp('start_pose_file'):
            with open(gp('start_pose_file')) as f:
                sp = (yaml.safe_load(f) or {}).get('start_pose') or {}
            x0, y0, yaw0 = float(sp.get('x', 0.0)), float(sp.get('y', 0.0)), float(sp.get('yaw', 0.0))
        self.plant.x, self.plant.y, self.plant.yaw = x0, y0, yaw0

        self.base_frame = gp('base_frame')
        self.imu_frame = gp('imu_frame')
        self.odom_frame = gp('odom_frame')
        self.report_frame = gp('frame_id')
        self.gyro_noise_std = float(gp('gyro_noise_std'))
        self.gyro_offset = tuple(float(gp(f'angular_velocity_offset_{a}')) for a in 'xyz')
        self.gyro_bias_error_z = float(gp('gyro_bias_error_z'))
        self.pwm_timeout = float(gp('pwm_timeout'))
        self.hold_on_loss = gp('on_pwm_loss') == 'hold'
        self.rng = random.Random()

        self.last_pwm_time = None
        self.pwm_lost = False
        self.q_base_imu = None    # orientation of the IMU frame in base_link
        self.last_step = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_vel = self.create_publisher(VelocityReport, gp('velocity_topic'), 1)
        self.pub_imu = self.create_publisher(Imu, gp('imu_topic'), 10)
        self.pub_odom = self.create_publisher(Odometry, '~/odometry', 10)
        self.create_subscription(Float32MultiArrayStamped, gp('pwm_topic'), self.on_pwm, 10)
        self.create_timer(1.0 / float(gp('physics_rate')), self.on_physics)
        self.create_timer(1.0 / float(gp('rate')), self.on_velocity_report)
        self.create_timer(1.0 / float(gp('imu_rate')), self.on_imu)
        self.create_timer(1.0 / float(gp('odom_rate')), self.on_odom)
        self.get_logger().info(
            f'motor {motor}; steering {steering}; IMU in {self.imu_frame}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------ input
    def on_pwm(self, msg):
        if len(msg.data) < 2:
            return
        self.plant.command(msg.data[0], msg.data[1])
        self.last_pwm_time = self.now_sec()
        if self.pwm_lost:
            self.get_logger().info('actuator PWM is back')
            self.pwm_lost = False

    # ------------------------------------------------------------ physics
    def on_physics(self):
        now = self.now_sec()
        if self.last_step is None:
            self.last_step = now
            return
        dt = min(max(now - self.last_step, 0.0), 0.05)
        self.last_step = now
        if (self.last_pwm_time is not None and not self.pwm_lost
                and now - self.last_pwm_time > self.pwm_timeout):
            self.pwm_lost = True
            if self.hold_on_loss:
                self.get_logger().warn(
                    f'no actuator PWM for {self.pwm_timeout} s: holding the last PWM '
                    f'(motor {self.plant.motor_pwm}, steering {self.plant.steer_pwm})')
            else:
                self.get_logger().warn(
                    f'no actuator PWM for {self.pwm_timeout} s: motor to neutral, steering centred')
                self.plant.command(self.plant.motor.params.init_pwm, self.plant.steering.init_steer)
        self.plant.step(dt)

    # ------------------------------------------------------------ outputs
    def on_velocity_report(self):
        msg = VelocityReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.report_frame
        msg.longitudinal_velocity = float(self.plant.wheel_speed_report())
        msg.lateral_velocity = 0.0
        msg.heading_rate = 0.0
        self.pub_vel.publish(msg)

    def lookup_imu_mount(self):
        if self.q_base_imu is not None:
            return True
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.imu_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().info(
                f'waiting for TF {self.base_frame} -> {self.imu_frame}; IMU not published yet',
                throttle_duration_sec=5.0)
            return False
        r = t.transform.rotation
        self.q_base_imu = (r.x, r.y, r.z, r.w)
        self.get_logger().info(f'IMU mount rotation in {self.base_frame}: {self.q_base_imu}')
        return True

    def on_imu(self):
        if not self.lookup_imu_mount():
            return
        p = self.plant
        q_inv = quat_conjugate(self.q_base_imu)
        gyro = (0.0, 0.0, p.yaw_rate + self.gyro_bias_error_z
                + self.rng.gauss(0.0, self.gyro_noise_std))
        accel = (p.accel, p.speed * p.yaw_rate, GRAVITY)
        wx, wy, wz = quat_rotate(q_inv, gyro)
        # imu_corrector subtracts its offsets in the IMU frame
        wx += self.gyro_offset[0]
        wy += self.gyro_offset[1]
        wz += self.gyro_offset[2]
        ax, ay, az = quat_rotate(q_inv, accel)
        qx, qy, qz, qw = quat_multiply(quat_from_yaw(p.yaw), self.q_base_imu)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation.x, msg.orientation.y = qx, qy
        msg.orientation.z, msg.orientation.w = qz, qw
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = wx, wy, wz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        var = max(self.gyro_noise_std, 1e-4) ** 2
        msg.angular_velocity_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        msg.orientation_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
        msg.linear_acceleration_covariance = [1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2]
        self.pub_imu.publish(msg)

    def on_odom(self):
        p = self.plant
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = p.x
        msg.pose.pose.position.y = p.y
        qx, qy, qz, qw = quat_from_yaw(p.yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = p.speed
        msg.twist.twist.angular.z = p.yaw_rate
        self.pub_odom.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleBenchPlant()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
