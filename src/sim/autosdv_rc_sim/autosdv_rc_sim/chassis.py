"""Single-track drift chassis (CommonRoad STD), rear-wheel drive. Completed in Task 11."""
from dataclasses import dataclass


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
