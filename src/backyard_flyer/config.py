# Missio constants

# Purpose: keep mission parameters in one place

from dataclasses import dataclass

# NOTE: frozen=True makes config immutable (safer)

@dataclass(frozen=True)
class MissionConfig:
    box_size_m: float = 10.0
    target_altitude_m: float = 3.0

    # tolerances
    pos_tolerance_m: float = 0.5
    vel_xy_tolerance_mps: float = 0.5
    ground_down_tolerance_m: float = 0.1

    # takeoff completion threshold
    takeoff_altitude_ratio: float = 0.95

    # heading to maintain during cmd_position
    heading_rad: float = 0.0
