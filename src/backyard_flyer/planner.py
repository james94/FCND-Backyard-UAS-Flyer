# Waypoint Planner (the 10m box)

# Purpose: generate waypoints relative to the current position

# from __future__ import annotations

from dataclasses import dataclass
from typing import List

from .config import MissionConfig
from .types import Waypoint

# Why class instead of a function?
    # planner can later support multiple patterns (triangle, lawnmower, etc.)
    # without changing the rest of the code.

@dataclass
class BoxPlanner:
    cfg: MissionConfig

    def build_box(self, origin_north: float, origin_east: float) -> List[Waypoint]:
        s = self.cfg.box_size_m

        return [
            Waypoint(origin_north + s, origin_east + 0.0, self.cfg.target_altitude_m),
            Waypoint(origin_north + s, origin_east + s, self.cfg.target_altitude_m),
            Waypoint(origin_north + 0.0, origin_east + s, self.cfg.target_altitude_m),
            Waypoint(origin_north + 0.0, origin_east + 0.0, self.cfg.target_altitude_m),
        ]
