"""Karnopp friction on the axles.

Each axle has a Coulomb torque (opposing rotation), a hold capacity (the
largest torque static friction resists at standstill) and a viscous term.
Rear: drivetrain Coulomb / breakaway (gear mesh, bearings, motor cogging) +
rolling resistance as a wheel torque + the ESC brake. Front: bearings +
rolling resistance.

Stick rule (Karnopp 1985): while |omega| < band and the torque acting on the
axle apart from its Coulomb friction is within the hold capacity, the axle is
held at zero. This is what makes a command that only *holds* a crawl unable to
*start* the car from rest.
"""
from dataclasses import dataclass

from autosdv_rc_sim.params import VehicleParams


@dataclass(frozen=True)
class WheelFriction:
    coulomb: float   # [N m] opposing rotation while turning
    hold: float      # [N m] static capacity at standstill
    viscous: float   # [N m s/rad]


def rear_friction(p: VehicleParams, fz_r: float, brake: float) -> WheelFriction:
    rolling = p.rolling_resistance * fz_r * p.wheel_radius
    braking = brake * p.brake_torque_max
    return WheelFriction(p.drivetrain_coulomb + rolling + braking,
                         p.drivetrain_breakaway + rolling + braking,
                         p.drivetrain_viscous)


def front_friction(p: VehicleParams, fz_f: float) -> WheelFriction:
    rolling = p.rolling_resistance * fz_f * p.wheel_radius
    return WheelFriction(p.front_bearing_coulomb + rolling, p.front_bearing_coulomb + rolling, 0.0)


def sticks(omega: float, external: float, friction: WheelFriction, band: float) -> bool:
    """Is the axle held this step?"""
    return abs(omega) < band and abs(external) <= friction.hold


def friction_sign(omega: float, external: float, band: float) -> float:
    """Direction the Coulomb torque opposes: the rotation, or inside the band the impending motion.

    Inside the band an axle that still turns against the external torque (a rolling wheel
    the tyre is slowing down) keeps opposing its rotation: taking the external torque's
    sign there would make the Coulomb torque drive the wheel along its rotation, and a car
    coasting to a stop would creep forever at the band's edge (|omega| = band).
    """
    if abs(omega) >= band or omega * external < 0.0:
        return 1.0 if omega > 0.0 else -1.0
    if external > 0.0:
        return 1.0
    if external < 0.0:
        return -1.0
    return 0.0
