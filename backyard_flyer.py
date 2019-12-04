import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # Flight parameters
        self.flight_square_side = 10.0
        self.flight_altitude = 3.0
        # NED (down will be negative)
        self.hover_position = [0.0, 0.0, self.flight_altitude]

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.transition)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.transition)
        self.register_callback(MsgID.STATE, self.transition)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        # NOTE: To have a better overview of "Phase" transitions and checks, I implemented positional checks in 
        # `transition()`.
        # If we want to improve performance, position dependent transitions can be triggered here instead,
        # but I don't think it's necessary to complicate the code now by adding a proper state machine.
        pass

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        # NOTE: To have a better overview of "Phase" transitions and checks, I implemented velocity checks in 
        # `transition()`.
        # If we want to improve performance, velocity dependent transitions can be triggered here instead,
        # but I don't think it's necessary to complicate the code now by adding a proper state machine.
        pass

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        # NOTE: This is triggered perioadically, even if `self.armed` or `self.guided` didn't change.
        # print('self.armed', self.armed)
        # print('self.guided', self.guided)

        # NOTE: To have a better overview of "Phase" transitions and checks, I implemented state checks in 
        # `transition()`.
        # If we want to improve performance, state dependent transitions can be triggered here instead,
        # but I don't think it's necessary to complicate the code now by adding a proper state machine.

    def calculate_box(self):
        """TODO: Fill out this method

        1. Return waypoints to fly a box
        """
        # [[North, East, Down]]
        return [
            # Top Left
            [self.flight_square_side, 0, self.flight_altitude],
            [self.flight_square_side, self.flight_square_side, self.flight_altitude],
            [0, self.flight_square_side, self.flight_altitude],
            # Bottom Left / Starting Position
            [0, 0, self.flight_altitude]
        ]

    def close_to(self, target_position):
        """Compare two positions with absolute tolerance of 0.1 units and 95%
        """
        current_position = [
            self.local_position[0],
            self.local_position[1],
            # Very non-intuitive, altitude is "negative" when flying up in NED (ok), even though we commanded it to fly
            # to a positive value (what?!)
            -self.local_position[2]
        ]
        return np.allclose(target_position, current_position, atol=0.1, rtol=0.05)

    def fly_to_position(self, position):
        """Command the drone to fly to a given position
        """
        print('Flying to', position)
        self.cmd_position(position[0], position[1], position[2], 0.0)

    def transition(self):
        """Transition through different states with appropriate checks.
        """
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.TAKEOFF:
            altitude = -self.local_position[2]
            target_altitude = self.target_position[2]
            if altitude / target_altitude >= 0.95:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            destination = self.all_waypoints[0]
            close_to_destination = self.close_to(destination)
            if close_to_destination:
                if len(self.all_waypoints) == 1:
                    self.landing_transition()
                else:
                    self.all_waypoints.pop(0)
                    self.fly_to_position(self.all_waypoints[0])
        elif self.flight_state == States.LANDING:
            # If we want to land on a specific spot (less than 0.3 accuracy), we need to correct for drift

            # Don't fly the drone into the ground, please!
            close_to_destination = -self.local_position[2] < 0.3
            if close_to_destination:
                self.disarming_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed and not self.guided:
                self.manual_transition()

    def arming_transition(self):
        """TODO: Fill out this method

        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(
            # North (lon?)
            self.global_position[0],
            # East (lat)
            self.global_position[1],
            # Down (alt)
            self.global_position[2]
        )
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method

        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[2] = self.hover_position[2]
        self.takeoff(self.hover_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method

        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        self.all_waypoints = self.calculate_box()
        self.flight_state = States.WAYPOINT
        self.fly_to_position(self.all_waypoints[0])

    def landing_transition(self):
        """TODO: Fill out this method

        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method

        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        # Why do these 2 methods need to be in the same function?!
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

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
