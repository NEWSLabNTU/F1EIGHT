"""Single-track drift chassis (CommonRoad "STD", vehicle_dynamics_std.py), rear-wheel drive.

State: CoM position and heading, body-frame velocities at the CoM, both axle
speeds and the rear wheel angle. CommonRoad's (v, beta) are hypot/atan2 of
(vx, vy); body velocities avoid STD's 1/v term in beta-dot. Below v_switch the
lateral and yaw states blend into the kinematic bicycle's at the step's new
speed (CommonRoad's tanh weight, applied to the states rather than through a
relaxation, so the yaw rate cannot outlive the speed as the car stops), and
the slip denominators never drop below v_min, so the model is stable at
standstill.

Integration (dt = 1 ms): the longitudinal subsystem (omega_f, omega_r, vx) is
stiff at low speed (wheel-slip time constant ~0.03 ms), so it is advanced by
linearly implicit Euler with a forward-difference Jacobian; Karnopp sticking
replaces an axle's row with omega = 0, and an axle whose slip changes sign
within the step ends it rolling (momentum-conserving contact stick), which
the linearisation cannot do where the tyre is saturated. Lateral and yaw use
explicit Euler, positions the new velocities.
"""
from dataclasses import dataclass
from typing import List

from autosdv_rc_sim.detmath import atan2, cos, PI, sin, tanh
from autosdv_rc_sim.params import GRAVITY, VehicleParams
from autosdv_rc_sim.tires import combined
from autosdv_rc_sim.wheels import friction_sign, front_friction, rear_friction, sticks


@dataclass(frozen=True)
class ChassisState:
    """Body-frame velocities at the CoM; (v, beta) of CommonRoad STD are hypot/atan2 of (vx, vy)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    omega_f: float = 0.0
    omega_r: float = 0.0
    theta_r: float = 0.0
    ax: float = 0.0
    ay: float = 0.0


@dataclass(frozen=True)
class Diagnostics:
    fz_f: float
    fz_r: float
    kappa_f: float
    kappa_r: float
    alpha_f: float
    alpha_r: float
    fx_f: float
    fy_f: float
    fx_r: float
    fy_r: float
    stuck_f: bool
    stuck_r: bool
    blend: float


def solve3(a: List[List[float]], b: List[float]) -> List[float]:
    """Gaussian elimination with partial pivoting (deterministic: ties keep the first row)."""
    a = [row[:] for row in a]
    b = b[:]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if a[pivot][col] == 0.0:
            raise ZeroDivisionError('singular longitudinal system')
        if pivot != col:
            a[col], a[pivot] = a[pivot], a[col]
            b[col], b[pivot] = b[pivot], b[col]
        for r in range(col + 1, 3):
            f = a[r][col] / a[col][col]
            for c in range(col, 3):
                a[r][c] -= f * a[col][c]
            b[r] -= f * b[col]
    x = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        acc = b[r]
        for c in range(r + 1, 3):
            acc -= a[r][c] * x[c]
        x[r] = acc / a[r][r]
    return x


def _wrap(angle: float) -> float:
    while angle > PI:
        angle -= 2.0 * PI
    while angle < -PI:
        angle += 2.0 * PI
    return angle


class Chassis:
    def __init__(self, p: VehicleParams):
        self.p = p
        self.last = None

    def loads(self, ax: float):
        p = self.p
        fz_f = p.mass * (GRAVITY * p.lr - ax * p.cg_height) / p.wheel_base
        fz_r = p.mass * (GRAVITY * p.lf + ax * p.cg_height) / p.wheel_base
        return max(fz_f, 0.0), max(fz_r, 0.0)

    def tire_forces(self, omega_f, omega_r, vx, vy, yaw_rate, cd, sd, fz_f, fz_r):
        p = self.p
        r = p.wheel_radius
        vlat_f = vy + p.lf * yaw_rate
        u_f = vx * cd + vlat_f * sd
        w_f = -vx * sd + vlat_f * cd
        w_r = vy - p.lr * yaw_rate
        den_f = max(abs(u_f), p.v_min)
        den_r = max(abs(vx), p.v_min)
        kf = (omega_f * r - u_f) / den_f
        kr = (omega_r * r - vx) / den_r
        af = -atan2(w_f, den_f)
        ar = -atan2(w_r, den_r)
        fxf, fyf = combined(kf, af, fz_f, p.mu, p.b_long, p.b_lat_front, p.pacejka_c, p.pacejka_e)
        fxr, fyr = combined(kr, ar, fz_r, p.mu, p.b_long, p.b_lat_rear, p.pacejka_c, p.pacejka_e)
        return fxf, fyf, fxr, fyr, kf, kr, af, ar

    def _longitudinal(self, z, s, cd, sd, fz_f, fz_r, tau, fr_f, fr_r, sgn_f, sgn_r):
        """(omega_f', omega_r', vx') at z = (omega_f, omega_r, vx); Coulomb signs held fixed."""
        p = self.p
        r = p.wheel_radius
        omega_f, omega_r, vx = z
        fxf, fyf, fxr, _, _, _, _, _ = self.tire_forces(
            omega_f, omega_r, vx, s.vy, s.yaw_rate, cd, sd, fz_f, fz_r)
        d_omega_f = (-r * fxf - fr_f.coulomb * sgn_f - fr_f.viscous * omega_f) / p.front_axle_inertia
        d_omega_r = (tau - r * fxr - fr_r.coulomb * sgn_r - fr_r.viscous * omega_r) / p.rear_axle_inertia
        fx_body = fxf * cd - fyf * sd + fxr - p.aero_drag * vx * abs(vx)
        return [d_omega_f, d_omega_r, fx_body / p.mass + s.vy * s.yaw_rate]

    def step(self, s: ChassisState, tau_drive: float, dtau_domega: float, brake: float,
             delta: float, dt: float) -> ChassisState:
        p = self.p
        r = p.wheel_radius
        cd = cos(delta)
        sd = sin(delta)
        fz_f, fz_r = self.loads(s.ax)
        fr_f = front_friction(p, fz_f)
        fr_r = rear_friction(p, fz_r, brake)
        fxf, fyf, fxr, fyr, kf, kr, af, ar = self.tire_forces(
            s.omega_f, s.omega_r, s.vx, s.vy, s.yaw_rate, cd, sd, fz_f, fz_r)

        # Karnopp: the torque on each axle apart from its Coulomb friction
        ext_f = -r * fxf - fr_f.viscous * s.omega_f
        ext_r = tau_drive - r * fxr - fr_r.viscous * s.omega_r
        # An axle sticks only if its hold capacity can carry the torque it would
        # see held still: the tyre force of a non-turning wheel. On a moving car
        # that is a skidding tyre, far beyond the hold, so a slowly rolling wheel
        # is never locked by the stick test (it would chatter as a brake).
        stuck_f = stuck_r = False
        if abs(s.omega_f) < p.stick_band:
            held = self.tire_forces(0.0, s.omega_r, s.vx, s.vy, s.yaw_rate, cd, sd, fz_f, fz_r)
            stuck_f = sticks(s.omega_f, -r * held[0], fr_f, p.stick_band)
        if abs(s.omega_r) < p.stick_band:
            held = self.tire_forces(s.omega_f, 0.0, s.vx, s.vy, s.yaw_rate, cd, sd, fz_f, fz_r)
            stuck_r = sticks(s.omega_r, tau_drive - r * held[2], fr_r, p.stick_band)
        sgn_f = 0.0 if stuck_f else friction_sign(s.omega_f, ext_f, p.stick_band)
        sgn_r = 0.0 if stuck_r else friction_sign(s.omega_r, ext_r, p.stick_band)

        # Longitudinal: linearly implicit Euler on (omega_f, omega_r, vx)
        z0 = [s.omega_f, s.omega_r, s.vx]
        args = (s, cd, sd, fz_f, fz_r, tau_drive, fr_f, fr_r, sgn_f, sgn_r)
        f0 = self._longitudinal(z0, *args)
        jac = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        for i in range(3):
            h = 1e-6 * max(1.0, abs(z0[i]))
            zi = list(z0)
            zi[i] += h
            fi = self._longitudinal(zi, *args)
            for row in range(3):
                jac[row][i] = (fi[row] - f0[row]) / h
        jac[1][1] += dtau_domega / p.rear_axle_inertia
        a = [[(1.0 if row == col else 0.0) - dt * jac[row][col] for col in range(3)]
             for row in range(3)]
        b = [dt * f0[0], dt * f0[1], dt * f0[2]]
        if stuck_f:
            a[0], b[0] = [1.0, 0.0, 0.0], -s.omega_f
        if stuck_r:
            a[1], b[1] = [0.0, 1.0, 0.0], -s.omega_r
        d = solve3(a, b)
        omega_f = s.omega_f + d[0]
        omega_r = s.omega_r + d[1]
        vx = s.vx + d[2]

        # Contact stick: a slip that changes sign within the step means the tyre
        # could have stopped sliding; end the step rolling, conserving momentum.
        vlat_f = s.vy + p.lf * s.yaw_rate
        sigma0_f = s.omega_f * r - (s.vx * cd + vlat_f * sd)
        u_f1 = vx * cd + vlat_f * sd
        if not stuck_f and sigma0_f * (omega_f * r - u_f1) < 0.0:
            dw = (u_f1 / r - omega_f) / (1.0 + cd * cd * p.front_axle_inertia / (p.mass * r * r))
            omega_f += dw
            vx -= cd * p.front_axle_inertia * dw / (r * p.mass)
        if not stuck_r and (s.omega_r * r - s.vx) * (omega_r * r - vx) < 0.0:
            dw = (vx / r - omega_r) / (1.0 + p.rear_axle_inertia / (p.mass * r * r))
            omega_r += dw
            vx -= p.rear_axle_inertia * dw / (r * p.mass)

        # Lateral and yaw: STD, blended into the kinematic bicycle below v_switch
        fy_body = fxf * sd + fyf * cd + fyr
        mz = p.lf * (fxf * sd + fyf * cd) - p.lr * fyr
        dvy_std = fy_body / p.mass - s.vx * s.yaw_rate
        dr_std = mz / p.yaw_inertia
        # The kinematic bicycle at the new speed: no relaxation lag, so the yaw
        # rate follows the speed down to standstill (a lag would let the yaw
        # rate outlive the speed when the car stops faster than the lag)
        r_ks = vx * (sd / cd) / p.wheel_base
        vy_ks = p.lr * r_ks
        w = 0.5 * (tanh((abs(vx) - p.v_switch) / p.v_blend) + 1.0)
        vy = w * (s.vy + dt * dvy_std) + (1.0 - w) * vy_ks
        yaw_rate = w * (s.yaw_rate + dt * dr_std) + (1.0 - w) * r_ks

        yaw = _wrap(s.yaw + dt * yaw_rate)
        cy = cos(yaw)
        sy = sin(yaw)
        self.last = Diagnostics(fz_f, fz_r, kf, kr, af, ar, fxf, fyf, fxr, fyr, stuck_f, stuck_r, w)
        return ChassisState(
            x=s.x + dt * (vx * cy - vy * sy),
            y=s.y + dt * (vx * sy + vy * cy),
            yaw=yaw, vx=vx, vy=vy, yaw_rate=yaw_rate,
            omega_f=omega_f, omega_r=omega_r,
            theta_r=s.theta_r + dt * omega_r,
            ax=(vx - s.vx) / dt - s.vy * s.yaw_rate,
            ay=fy_body / p.mass,
        )
