# Isolate UdaciDrone API calls

# Purpose: the FSM should not care whether it's talking to a simulator, real drone, or a mock.

# This adapter is a thin wrapper around the underlying "Drone" object.

# from __future__ import annotations

from dataclasses import dataclass

# python 3.6 vs 3.8
try:
    # python 3.8
    from typing import Protocol
except ImportError:
    # python 3.6
    from typing_extensions import Protocol

# NOTE: the Protocol makes it easy to unit test FSM by passing a mock object
    # This adapter is where API-version differences can be handled

class DroneLike(Protocol):
    # attributes
    armed: bool
    guided: bool
    local_position: object
    local_velocity: object
    global_position: object

    # commands
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def take_control(self) -> None: ...
    def release_control(self) -> None: ...
    def takeoff(self, target_altitude: float) -> None: ...
    def land(self) -> None: ...
    def cmd_position(self, north: float, east: float, altitude: float, heading: float) -> None: ...
    def set_home_position(self, lon: float, lat: float, alt: float) -> None: ...
    def stop(self) -> None: ...

@dataclass
class UdacidroneAdapter:
    drone: DroneLike

    def take_control_and_arm(self) -> None:
        self.drone.take_control()
        self.drone.arm()

        # Match common Udacity solutions: set home to current global position.
        # This helps ensure the local frame behaves as expected.
        try:
            gp = self.drone.global_position
            self.drone.set_home_position(gp[0], gp[1], gp[2])
        except Exception:
            pass

    def takeoff(self, altitude_m: float) -> None:
        self.drone.takeoff(altitude_m)

    def goto(self, north: float, east: float, altitude_m: float, heading: float) -> None:
        self.drone.cmd_position(north, east, altitude_m, heading)

    def land(self) -> None:
        self.drone.land()

    def disarm(self) -> None:
        self.drone.disarm()

    def finish(self) -> None:
        self.drone.release_control()
        self.drone.stop()

    