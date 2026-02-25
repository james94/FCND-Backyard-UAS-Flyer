# the state machine class

# Purpose: store state, manage waypoints, and call adapter commands

# We'll mirror the project's states

# from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

import numpy as np

from .adapter import UdacidroneAdapter
from .config import MissionConfig
from .guards import FlightGuards
from .planner import BoxPlanner
from .types import NED, Waypoint

class FlightState(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

# What to notice:
    # Callback inputs (on_local_position, on_state_update, on_local_velocity) are pure data
    # Transitions are the only place that sends commands
    # BackyardFlyerFSM is now testable with a fake adapter

@dataclass
class BackyardFlyerFSM:
    cfg: MissionConfig
    adapter: UdacidroneAdapter
    planner: BoxPlanner
    guards: FlightGuards

    state: FlightState = FlightState.MANUAL
    in_mission: bool = True

    target: Optional[Waypoint] = None
    waypoints: List[Waypoint] = field(default_factory=list)

    def on_state_update(self, armed: bool, guided: bool) -> None:
        if self.state == FlightState.MANUAL and self.in_mission: 
            self._arming_transition()

            return

        if self.state == FlightState.ARMING and armed:
            self._takeoff_transition()

            return
        
        if self.state == FlightState.DISARMING and not armed:
            self._manual_transition()

            return

    def on_local_position(self, local_position: np.ndarray) -> None:
        if self.state == FlightState.TAKEOFF and self.guards.altitude_reached(local_position):
            if not self.waypoints:
                self.waypoints = self.planner.build_box(
                    origin_north=float(local_position[0]),
                    origin_east=float(local_position[1]),
                )
            
            self._waypoint_transition()

            return
        
        if self.state == FlightState.WAYPOINT and self.target is not None:
            if self.guards.waypoint_reached(local_position, self.target):
                if self.waypoints:
                    self._waypoint_transition()
                else:
                    self._landing_transition()
            
        
    def on_local_velocity(self, local_position: np.ndarray, local_velocity: np.ndarray) -> None:
        if self.state == FlightState.LANDING:
            if self.guards.ready_to_disarm(local_position, local_velocity):
                self._disarming_transition()
    
    # --- transitions (actions) ---

    def _arming_transition(self) -> None:
        print("arming transition")
        self.adapter.take_control_and_arm()
        self.state = FlightState.ARMING

    def _takeoff_transition(self) -> None:
        print("takeoff transition")
        self.adapter.takeoff(self.cfg.target_altitude_m)
        self.state = FlightState.TAKEOFF

    def _waypoint_transition(self) -> None:
        print("waypoint transition")
        self.target = self.waypoints.pop(0) if self.waypoints else None
        
        if self.target is None:
            self._landing_transition()
            return

        # UdaciDrone cmd_position expects altitude-up (meters), not NED down.
        self.adapter.goto(
            north=self.target.north,
            east=self.target.east,
            altitude_m=self.target.altitude_m,
            heading=self.cfg.heading_rad,
        )

        self.state = FlightState.WAYPOINT
    
    def _landing_transition(self) -> None:
        print("landing transition")
        self.adapter.land()
        self.state = FlightState.LANDING

    def _disarming_transition(self) -> None:
        print("disarm transition")
        self.adapter.disarm()
        self.state = FlightState.DISARMING
    
    def _manual_transition(self) -> None:
        print("manual transition")
        self.adapter.finish()
        self.in_mission = False
        self.state = FlightState.MANUAL
