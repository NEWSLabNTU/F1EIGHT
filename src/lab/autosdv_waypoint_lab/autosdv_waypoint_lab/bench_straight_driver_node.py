"""bench_straight_driver: the TA's transmitter for `just lab2 survey-align --bench`. BENCH ONLY.

On the field the TA drives the car straight along the taped first leg by
transmitter; on the bench this node does it to the simulated car. It writes the
PWM pair the bench plant (autosdv_coach_lab vehicle_bench_plant) reads, the topic
the actuator would publish:

    /survey_align/state "ready"  (latched, from survey-align)
      -> wait `rest` s (the start pose is averaged over the car at rest)
      -> /autosdv/actuator_node/debug/pwm_values [motor, init_steer]: a PI loop on
         the plant's TRUE speed holds `speed`, steering centred, for `distance` m
      -> brake, then neutral; ~/done (std_msgs/Bool, latched) true

It refuses to run (and exits) unless it is on a bench: the plant's true pose
(/vehicle_bench_plant/odometry) must be published, and nothing else may publish
the PWM topic -- on the car the actuator does, and this node writes nothing.
(The actuator's debug topic does not drive a car in any case: the PCA9685 is
written by the actuator alone.)
"""

import math

import rclpy
from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class BenchStraightDriver(Node):
    def __init__(self):
        super().__init__('bench_straight_driver')
        p = self.declare_parameter
        p('distance', 3.0)          # m
        p('speed', 0.3)             # m/s
        p('rest', 2.0)              # s at rest after "ready"
        p('init_pwm', 370)
        p('init_steer', 489)
        p('brake_pwm', 340)
        p('deadband', 5)            # PWM counts: MotorPlantParams' defaults, for the feedforward
        p('gain', 0.03)             # m/s per count
        p('kp', 40.0)
        p('ki', 20.0)
        p('pwm_topic', '/autosdv/actuator_node/debug/pwm_values')
        p('true_pose_topic', '/vehicle_bench_plant/odometry')
        p('state_topic', '/survey_align/state')
        p('bench_check_s', 8.0)
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.distance, self.speed, self.rest = float(gp('distance')), float(gp('speed')), float(gp('rest'))
        self.init_pwm, self.init_steer = int(gp('init_pwm')), int(gp('init_steer'))
        self.brake_pwm = int(gp('brake_pwm'))
        self.ff = self.init_pwm + int(gp('deadband')) + self.speed / float(gp('gain'))
        self.kp, self.ki = float(gp('kp')), float(gp('ki'))
        self.pwm_topic = gp('pwm_topic')
        self.true_topic = gp('true_pose_topic')
        self.bench_check_s = float(gp('bench_check_s'))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Float32MultiArrayStamped, self.pwm_topic, 10)
        self.pub_done = self.create_publisher(Bool, '~/done', latched)
        self.create_subscription(Odometry, self.true_topic, self.on_odom, 10)
        self.create_subscription(String, gp('state_topic'), self.on_state, latched)
        self.phase = 'check'        # check -> wait -> rest -> drive -> brake -> done
        self.t0 = self.now()
        self.t_phase = self.t0
        self.pose = None
        self.v = 0.0
        self.start = None
        self.integral = 0.0
        self.ready = False
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'bench only: will drive {self.distance:g} m at {self.speed:g} m/s '
                               'after survey-align says ready')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y)
        self.v = m.twist.twist.linear.x

    def on_state(self, m):
        if m.data == 'ready':
            self.ready = True

    def send(self, motor, steer):
        msg = Float32MultiArrayStamped()
        msg.stamp = self.get_clock().now().to_msg()
        msg.data = [float(motor), float(steer)]
        self.pub.publish(msg)

    def go(self, phase):
        self.phase, self.t_phase = phase, self.now()
        self.get_logger().info(phase)

    def tick(self):
        t = self.now()
        if self.phase == 'check':
            others = self.count_publishers(self.pwm_topic) - 1
            if others > 0:
                self.get_logger().error(f'REFUSED: something else publishes {self.pwm_topic} (the '
                                        'actuator: this is not the bench). Writing nothing.')
                raise SystemExit(1)
            if self.pose is not None:
                self.go('wait')
            elif t - self.t0 > self.bench_check_s:
                self.get_logger().error(f'REFUSED: no {self.true_topic} (the bench plant) after '
                                        f'{self.bench_check_s:g} s: this is not the bench. Writing nothing.')
                raise SystemExit(1)
            return
        if self.phase == 'wait':
            self.send(self.init_pwm, self.init_steer)
            if self.ready:
                self.go('rest')
            return
        if self.phase == 'rest':
            self.send(self.init_pwm, self.init_steer)
            if t - self.t_phase >= self.rest:
                self.start = self.pose
                self.go('drive')
            return
        if self.phase == 'drive':
            d = math.hypot(self.pose[0] - self.start[0], self.pose[1] - self.start[1])
            if d >= self.distance:
                self.go('brake')
                return
            err = self.speed - self.v
            self.integral = max(-1.0, min(1.0, self.integral + err * 0.05))
            pwm = self.ff + self.kp * err + self.ki * self.integral
            self.send(max(self.init_pwm, min(self.init_pwm + 40, round(pwm))), self.init_steer)
            return
        if self.phase == 'brake':
            stopped = abs(self.v) < 0.01
            self.send(self.init_pwm if stopped else self.brake_pwm, self.init_steer)
            if stopped and t - self.t_phase > 0.3:
                d = math.hypot(self.pose[0] - self.start[0], self.pose[1] - self.start[1])
                self.get_logger().info(f'stopped after {d:.2f} m')
                self.pub_done.publish(Bool(data=True))
                self.go('done')
            return
        self.send(self.init_pwm, self.init_steer)


def main(args=None):
    rclpy.init(args=args)
    node = BenchStraightDriver()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
