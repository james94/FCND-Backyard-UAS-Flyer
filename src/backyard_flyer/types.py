# small, explicit data types

# Purpose: reduce "mystery arrays" and make intent clear

# from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

@dataclass(frozen=True)
class NED:
    north: float
    east: float
    down: float

    def as_np(self) -> np.ndarray:
        return np.array([self.north, self.east, self.down], dtype=float)
    
    @staticmethod
    def from_iterable(values: Iterable[float]) -> "NED":
        v = list(values)

        return NED(float(v[0]), float(v[1]), float(v[2]))


@dataclass(frozen=True)
class Waypoint:
    """Waypoint in local N/E with altitude-up (meters).

    Note: UdaciDrone's cmd_position() takes altitude-up, while local_position is NED.
    """

    north: float
    east: float
    altitude_m: float
