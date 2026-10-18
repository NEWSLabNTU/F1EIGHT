"""IMU sample for the interactive simulator (not on the deterministic grading path).

Body-frame yaw rate and accelerations at the CoM, plus a z-gyro bias and white
noise. The ROS node rotates them into the IMU frame and adds the static offsets
imu_corrector removes.
"""
import random
from typing import Tuple

from autosdv_rc_sim.params import VehicleParams


class ImuModel:
    def __init__(self, p: VehicleParams, seed: int = 0):
        self.p = p
        self.rng = random.Random(seed)

    def sample(self, state) -> Tuple[float, float, float]:
        p = self.p
        return (state.yaw_rate + p.gyro_bias_z + self.rng.gauss(0.0, p.gyro_noise_std),
                state.ax + self.rng.gauss(0.0, p.accel_noise_std),
                state.ay + self.rng.gauss(0.0, p.accel_noise_std))
