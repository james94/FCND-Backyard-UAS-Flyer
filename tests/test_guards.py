import numpy as np

from backyard_flyer.config import MissionConfig
from backyard_flyer.guards import FlightGuards
from backyard_flyer.types import NED

def test_altitude_reached_uses_ned_down_sign():
    cfg = MissionConfig(target_altitude_m=3.0, takeoff_altitude_ratio=0.95)
    guards = FlightGuards(cfg)

    # at altitude 3m => down = -3

    assert guards.altitude_reached(np.array([0.0, 0.0, -3.0]))


def test_waypoint_reached_xy_only():
    cfg = MissionConfg(pos_tolerance_m=0.5)
    guards = FlightGuards(cfg)
    target = NED(10.0, 0.0, -3.0)

    assert guards.waypoint_reached(np.array([10.2, 0.1, -3.0]), target)
