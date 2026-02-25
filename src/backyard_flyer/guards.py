# state transitions "if" logic

# Purpose: pure checks that are easy to unit test
# from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import MissionConfig
from .types import Waypoint

@dataclass
class FlightGuards:
    cfg: MissionConfig

    def altitude_reached(self, local_position: np.ndarray) -> bool:
        # local_position is NED => altitude = -down
        altitude = -float(local_position[2])
        return altitude >= self.cfg.takeoff_altitude_ratio * self.cfg.target_altitude_m

    def waypoint_reached(self, local_position: np.ndarray, target: Waypoint) -> bool:
        pos_ne = np.array([local_position[0], local_position[1]], dtype=float)
        tgt_ne = np.array([target.north, target.east], dtype=float)
        d = float(np.linalg.norm(pos_ne - tgt_ne))

        return d <= self.cfg.pos_tolerance_m

    def ready_to_disarm(self, local_position: np.ndarray, local_velocity: np.ndarray) -> bool:
        near_ground = abs(float(local_position[2])) <= self.cfg.ground_down_tolerance_m
        vel_xy = np.linalg.norm(np.array([local_velocity[0], local_velocity[1]], dtype=float))
        nearly_stopped = float(vel_xy) <= self.cfg.vel_xy_tolerance_mps

        return near_ground and nearly_stopped