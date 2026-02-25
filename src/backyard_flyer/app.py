# Wiring (build objects once)

# Purpose: central place to construct "cfg", planner, guards, FSM

# from __future__ import annotations

from .adapter import UdacidroneAdapter
from .config import MissionConfig
from .fsm import BackyardFlyerFSM
from .guards import FlightGuards
from .planner import BoxPlanner

def build_fsm(drone) -> BackyardFlyerFSM:
    cfg = MissionConfig()
    adapter = UdacidroneAdapter(drone=drone)
    planner = BoxPlanner(cfg=cfg)
    guards = FlightGuards(cfg=cfg)

    return BackyardFlyerFSM(cfg=cfg, adapter=adapter, planner=planner, guards=guards)
    
