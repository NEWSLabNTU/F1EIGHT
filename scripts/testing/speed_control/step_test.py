#!/usr/bin/env python3
"""Speed step test for the AutoSDV actuator, with abort guards for ground runs.

Publishes stamped /control/command/control_cmd at 50 Hz following a step
profile, logs speed / PWM / PID / mode / ESC state to CSV, prints metrics.

Guards (any one ends the run with a zero command):
  max run   -- the whole profile may not exceed --max-run seconds
  overspeed -- measured |speed| above --overspeed in --overspeed-reports
               consecutive velocity reports (one late edge timestamp makes a
               short gap, and the report's EMA carries that spike over about
               two reports; a real overspeed persists)
  stall     -- motor PWM at/above --stall-pwm while speed < 0.05 m/s for
               --stall-time (no speed feedback: lifted car, unplugged sensor)
  no report -- no velocity report for 0.5 s
  distance  -- integrated measured distance reaches --max-distance minus
               --stop-margin (room left to stop)
--ramp A limits how fast the command follows the profile [m/s^2], as
Autoware's planner does; 0 = steps. An abort still commands 0 at once.
After the profile or an abort, commands 0 for 2 s; when this process stops
publishing, the actuator's own watchdog brakes and returns to neutral.
"""
import argparse, csv, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport
from std_msgs.msg import String
from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped


def parse_profile(text):
    """'0:2,0.3:4,0:2' -> [(2.0, 0.0), (4.0, 0.3), (2.0, 0.0)] as (duration, target)."""
    out = []
    for item in text.split(','):
        v, d = item.split(':')
        out.append((float(d), float(v)))
    return out


class StepTest(Node):
    def __init__(self, a):
        super().__init__('pid_step_test')
        self.a = a
        self.profile = parse_profile(a.profile)
        self.pub = self.create_publisher(Control, '/control/command/control_cmd', 1)
        self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.on_vel, 10)
        self.create_subscription(Float32MultiArrayStamped, '/autosdv/actuator_node/debug/pwm_values', self.on_pwm, 10)
        self.create_subscription(Float32MultiArrayStamped, '/autosdv/actuator_node/debug/pid_values', self.on_pid, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/autosdv/actuator_node/debug/mode', self.on_mode, latched)
        self.create_subscription(String, '/autosdv/actuator_node/debug/esc_state', self.on_esc, latched)
        self.speed = math.nan; self.pwm = math.nan; self.pid = [math.nan] * 3
        self.mode = ''; self.esc = ''
        self.last_report = None; self.stall_since = None; self.distance = 0.0
        self.over_reports = 0
        self.rows = []; self.abort = None; self.done = False
        self.t0 = None; self.stop_until = None
        self.command = 0.0; self.last_tick = None
        self.create_timer(0.02, self.tick)

    def on_vel(self, m):
        now = time.monotonic()
        if self.last_report is not None and math.isfinite(self.speed):
            self.distance += abs(self.speed) * (now - self.last_report)
        self.speed = m.longitudinal_velocity; self.last_report = now
        self.over_reports = self.over_reports + 1 if abs(self.speed) > self.a.overspeed else 0
    def on_pwm(self, m): self.pwm = m.data[0]
    def on_pid(self, m): self.pid = list(m.data[4:7])  # speed_p, speed_i, speed_d
    def on_mode(self, m): self.mode = m.data
    def on_esc(self, m): self.esc = m.data

    def guard(self, now, t):
        a = self.a
        if t > a.max_run:
            return f'max run {a.max_run:.0f} s reached'
        if self.last_report is None:
            if t > 2.0:
                return 'no velocity report at all'
        elif now - self.last_report > 0.5:
            return 'velocity report lost (>0.5 s)'
        if self.distance >= a.max_distance - a.stop_margin:
            return f'distance {self.distance:.2f} m reached (limit {a.max_distance} m, {a.stop_margin} m to stop)'
        if self.over_reports >= a.overspeed_reports:
            return f'overspeed {self.speed:.2f} m/s > {a.overspeed} m/s for {self.over_reports} reports'
        if self.pwm >= a.stall_pwm and abs(self.speed) < 0.05:
            self.stall_since = self.stall_since or now
            if now - self.stall_since > a.stall_time:
                return f'stall: PWM {self.pwm:.0f} >= {a.stall_pwm} with no speed for {a.stall_time} s'
        else:
            self.stall_since = None
        return None

    def tick(self):
        now = time.monotonic()
        if self.t0 is None: self.t0 = now
        t = now - self.t0
        tgt = 0.0
        if self.stop_until is None:
            reason = self.guard(now, t)
            acc = 0.0; tgt = None
            for d, v in self.profile:
                if t < acc + d: tgt = v; break
                acc += d
            if reason or tgt is None:
                if reason:
                    self.abort = reason
                    self.get_logger().error(f'ABORT: {reason}')
                self.stop_until = now + 2.0; tgt = 0.0
        elif now >= self.stop_until:
            self.done = True; return
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        if self.a.ramp > 0.0 and not self.abort:
            step = self.a.ramp * dt
            self.command += max(-step, min(step, tgt - self.command))
        else:
            self.command = tgt
        msg = Control()
        msg.stamp = self.get_clock().now().to_msg()
        msg.longitudinal.velocity = float(self.command)
        msg.lateral.steering_tire_angle = 0.0
        self.pub.publish(msg)
        self.rows.append([round(t, 3), tgt, self.speed, self.pwm, *self.pid, self.mode, self.esc,
                          round(self.command, 4)])


def metrics(rows, profile):
    out = []; acc = 0.0
    for d, v in profile:
        seg = [r for r in rows if acc <= r[0] < acc + d]
        t_start = acc; acc += d
        if not seg or (v == 0.0 and t_start == 0.0):
            continue
        tail = [r for r in seg if r[0] >= t_start + d - min(2.0, d / 2)]
        ts = [r[2] for r in tail if not math.isnan(r[2])]; tp = [r[3] for r in tail if not math.isnan(r[3])]
        if not ts or not tp:
            continue
        mean = sum(ts) / len(ts); std = (sum((x - mean) ** 2 for x in ts) / len(ts)) ** 0.5
        rise = next((r[0] - t_start for r in seg if v > 0 and abs(r[2] - v) <= 0.1 * v), None)
        settle0 = next((r[0] - t_start for r in seg if v == 0 and abs(r[2]) < 0.05), None)
        peak = max(r[2] for r in seg) if v > 0 else None
        out.append(dict(target=v, mean=mean, err=mean - v, std=std, rise=rise, settle0=settle0,
                        peak=peak, pwm_mean=sum(tp) / len(tp), pwm_min=min(tp), pwm_max=max(tp)))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('out')
    p.add_argument('--profile', default='0:2,0.3:3,0.5:3,0:3', help='target:seconds,...')
    p.add_argument('--max-distance', type=float, default=3.0)
    p.add_argument('--stop-margin', type=float, default=0.3)
    p.add_argument('--max-run', type=float, default=20.0)
    p.add_argument('--overspeed', type=float, default=1.0)
    p.add_argument('--ramp', type=float, default=0.0,
                   help='command slew limit [m/s^2], as the planner ramps speed; 0 = steps')
    p.add_argument('--overspeed-reports', type=int, default=3,
                   help='consecutive velocity reports above --overspeed that abort (20 Hz: 3 = 150 ms)')
    p.add_argument('--stall-pwm', type=float, default=388.0)
    p.add_argument('--stall-time', type=float, default=1.5)
    a = p.parse_args()
    rclpy.init()
    n = StepTest(a)
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        with open(a.out, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['t', 'target', 'speed', 'pwm', 'p', 'i', 'd', 'mode', 'esc', 'command'])
            w.writerows(n.rows)
        print(('ABORTED: ' + n.abort if n.abort else 'completed') + f'  distance={n.distance:.2f} m')
        for m in metrics(n.rows, n.profile):
            print(' '.join(f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}' for k, v in m.items()))
        n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
