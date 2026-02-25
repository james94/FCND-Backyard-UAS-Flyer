# Hooking our modular class based Python approach into Udacity Starter

# We need a "Drone" subclass for UdaciDrone callbacks, but it can be
    # extremely thin:

# 1. Pattern: "BackyardFlyer(Drone)" delegates to BackyardFlyerFSM

    # 1. Build the FSM once in __init__
    # 2. In each callback, forward the updated attributes

import argparse
import time
# from enum import Enum

# import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

from src.backyard_flyer.app import build_fsm

# class States(Enum):
#     MANUAL = 0
#     ARMING = 1
#     TAKEOFF = 2
#     WAYPOINT = 3
#     LANDING = 4
#     DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.fsm = build_fsm(self)

        # NOTE: initial state and other attributes defined in FSM

        # self.target_position = np.array([0.0, 0.0, 0.0])
        # self.all_waypoints = []
        # self.in_mission = True
        # self.check_state = {}
        # self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        self.fsm.on_local_position(self.local_position)

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        self.fsm.on_local_velocity(self.local_position, self.local_velocity)

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        self.fsm.on_state_update(self.armed, self.guided)

    # NOTE: I believe this operation is already taken care of in local_position_callback,
        # it uses BoxPlanner's build_box(...) method to get waypoints
    # def calculate_box(self):
    #     """TODO: Fill out this method
        
    #     1. Return waypoints to fly a box
    #     """
    #     pass

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        self.fsm._arming_transition()

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        self.fsm._takeoff_transition()

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.fsm._waypoint_transition()

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        self.fsm._landing_transition()

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        self.fsm._disarming_transition()

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        # print("manual transition")

        # self.release_control()
        # self.stop()
        # self.in_mission = False
        # self.flight_state = States.MANUAL

        self._manual_transition()

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
