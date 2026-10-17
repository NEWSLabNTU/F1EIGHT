"""DC motor through a fixed gear onto the rear axle, with battery sag.

    I   = (d V - Ke G w) / (R_m + d^2 R_b)       (duty d != 0)
    I   = -s Ke G w / R_m                         (neutral, drag brake s)
    tau = G Kt I                                  at the rear axle

The battery delivers d I through R_b, so its terminal voltage is V - d I R_b;
that sag is the d^2 R_b term. The electrical time constant (under 1 ms) is
dropped. With the mock motor (Kv 3000, G 14.5, 0.03 + 0.03 ohm) full throttle
at standstill would draw 185 A. The ESC limits the current: esc_current_limit
(120 A) while it drives the car, and esc_regen_current_limit while it opposes
the duty, which is how a synchronous-rectifying ESC brakes when the duty falls
below the back-EMF, and what the drag brake does. The regeneration cap is no
more than the ESC's full brake (brake_torque_max), so motor braking cannot
lock the rear wheels where the brake does not. Full throttle at standstill
gives 105 N at the ground: far above the rear tyres' grip, so full throttle
from rest spins the rear wheels, as RC cars do.

The module functions are steady-state, no-slip figures for tests and the
track generator; the simulation itself never uses them.
"""
from typing import Tuple

from autosdv_rc_sim.params import GRAVITY, VehicleParams


class Drivetrain:
    def __init__(self, p: VehicleParams):
        self.p = p
        self.current = 0.0
        self.terminal_voltage = p.battery_voltage

    def torque(self, duty: float, short: float, omega_wheel: float) -> Tuple[float, float]:
        """Rear-axle torque [N m] and its derivative with respect to the axle speed."""
        p = self.p
        g = p.gear_ratio
        emf = p.ke * g * omega_wheel
        if duty != 0.0:
            r_total = p.motor_resistance + duty * duty * p.battery_resistance
            current = (duty * p.battery_voltage - emf) / r_total
            slope = -p.ke * g / r_total
            # Motoring (current along the duty) or regenerating (against it)
            limit = p.esc_current_limit if current * duty > 0.0 else p.esc_regen_current_limit
            if current > limit:
                current, slope = limit, 0.0
            elif current < -limit:
                current, slope = -limit, 0.0
            self.terminal_voltage = p.battery_voltage - duty * current * p.battery_resistance
        elif short > 0.0:
            current = -short * emf / p.motor_resistance
            slope = -short * p.ke * g / p.motor_resistance
            if current > p.esc_regen_current_limit:
                current, slope = p.esc_regen_current_limit, 0.0
            elif current < -p.esc_regen_current_limit:
                current, slope = -p.esc_regen_current_limit, 0.0
            self.terminal_voltage = p.battery_voltage
        else:
            current, slope = 0.0, 0.0
            self.terminal_voltage = p.battery_voltage
        self.current = current
        k = g * p.kt
        return k * current, k * slope


def drive_force(p: VehicleParams, duty: float, v: float) -> float:
    """Steady drive force at the ground [N] at speed v, no tyre slip."""
    tau, _ = Drivetrain(p).torque(duty, 0.0, v / p.wheel_radius)
    return tau / p.wheel_radius


def road_load(p: VehicleParams, v: float) -> float:
    """Resistance [N] while rolling forward at v >= 0 with the ESC at neutral (no drag brake)."""
    r = p.wheel_radius
    return (p.rolling_resistance * p.mass * GRAVITY
            + (p.drivetrain_coulomb + p.front_bearing_coulomb) / r
            + p.drivetrain_viscous * v / (r * r)
            + p.aero_drag * v * v)


def top_speed(p: VehicleParams, duty: float = 1.0) -> float:
    """Steady speed where drive force meets road load (bisection; no tyre slip)."""
    lo, hi = 0.0, duty * p.battery_voltage / (p.ke * p.gear_ratio) * p.wheel_radius
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if drive_force(p, duty, mid) > road_load(p, mid):
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def brake_decel(p: VehicleParams, v: float, level: float = 1.0) -> float:
    """Deceleration [m/s^2] under ESC brake `level` at speed v (no wheel lock)."""
    return (level * p.brake_torque_max / p.wheel_radius + road_load(p, v)) / p.effective_mass


def grip_accel(p: VehicleParams) -> float:
    """Largest launch acceleration the rear tyres allow, with load transfer, from rest."""
    rear_grip_static = p.mu * p.mass * GRAVITY * p.lf / p.wheel_base
    transfer = p.mu * p.mass * p.cg_height / p.wheel_base
    return (rear_grip_static - road_load(p, 0.0)) / (p.effective_mass - transfer)


def rear_brake_limit(p: VehicleParams, v: float = 0.0) -> float:
    """Largest deceleration with the rear tyres at their peak; the front only rolls.

    Braking unloads the rear axle (load transfer), which is why a rear-braked
    car stops worse than mu g.
    """
    r = p.wheel_radius
    front = p.rolling_resistance * p.fz_front_static + p.front_bearing_coulomb / r
    num = p.mu * p.mass * GRAVITY * p.lf / p.wheel_base + front + p.aero_drag * v * v
    den = p.mass + p.front_axle_inertia / (r * r) + p.mu * p.mass * p.cg_height / p.wheel_base
    return num / den
