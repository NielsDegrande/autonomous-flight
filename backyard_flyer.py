import argparse
import time
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


# Configuration.
ALTITUDE_ERROR = 0.05
DEFAULT_HEADING_IN_RADIANS = 0.0
DOWN_TO_ALTITUDE_CONVERSION = -1.0
LENGTH_BOX_SIDE_IN_METERS = 10.0
POSITION_ERROR = 0.01
POSITION_ERROR_IN_METER = 0.1
TARGET_ALTITUDE_IN_METERS = 3.0


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
        self.all_waypoints = self.calculate_box()
        self.in_mission = True

        # Initial state.
        self.flight_state = States.MANUAL

        # Register all your callbacks here
        self.register_callback(
            MsgID.LOCAL_POSITION, self.local_position_callback
        )
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """Act upon new position information.

        This triggers when `MsgID.LOCAL_POSITION` is received
        and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            # NOTE:
            #   Local position is defined as NED position.
            #   Convert down to altitude.
            altitude = DOWN_TO_ALTITUDE_CONVERSION * self.local_position[2]

            # Check if altitude is within 95% of target altitude.
            if altitude > (1 - ALTITUDE_ERROR) * self.target_position[2]:
                self.waypoint_transition()

        error_north = abs(self.local_position[0] - self.target_position[0])
        error_east = abs(self.local_position[1] - self.target_position[1])
        if (
            self.flight_state == States.WAYPOINT
            and error_north < POSITION_ERROR * LENGTH_BOX_SIDE_IN_METERS
            and error_east < POSITION_ERROR * LENGTH_BOX_SIDE_IN_METERS
        ):
            if self.all_waypoints:
                self.waypoint_transition()
            else:
                self.landing_transition()

    def velocity_callback(self):
        """Act upon new velocity information.
        
        When getting close to landing position: disarm.

        This triggers when `MsgID.LOCAL_VELOCITY` is received
        and self.local_velocity contains new data.
        """
        if (
            (self.flight_state == States.LANDING)
            and (
                self.global_position[2] - self.global_home[2]
                < POSITION_ERROR_IN_METER
            )
            and (abs(self.local_position[2]) < POSITION_ERROR_IN_METER)
        ):
            self.disarming_transition()

    def state_callback(self):
        """Act upon new state information.

        This triggers when `MsgID.STATE` is received
        and self.armed and self.guided contain new data.
        """
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        """Calculate box of waypoints.
        
        1. Return waypoints to fly a box.
        """
        waypoints = []
        for north in (LENGTH_BOX_SIDE_IN_METERS, 0):
            waypoints.append(
                (
                    north,
                    LENGTH_BOX_SIDE_IN_METERS - north,
                    TARGET_ALTITUDE_IN_METERS,
                )
            )
            waypoints.append((north, north, TARGET_ALTITUDE_IN_METERS,))
        return waypoints

    def arming_transition(self):
        """Arm the quadrotor.
        
        1. Take control of the drone.
        2. Pass an arming command.
        3. Set the home location to current position.
        4. Transition to the ARMING state.
        """
        print("Arming transition.")
        self.take_control()
        self.arm()
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """Take off with quadrotor.
        
        1. Set target_position altitude to 3.0m.
        2. Command a takeoff to 3.0m.
        3. Transition to the TAKEOFF state.
        """
        # Set the current location to be the home position before takeoff.
        print("Save home position.")
        self.set_home_position(
            self.global_position[0],
            self.global_position[1],
            self.global_position[2],
        )

        print("Takeoff transition.")
        self.target_position[2] = TARGET_ALTITUDE_IN_METERS
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """Navigate quadrotor towards a waypoint.
    
        1. Command the next waypoint position.
        2. Transition to WAYPOINT state.
        """
        print("Waypoint transition.")

        waypoint = self.all_waypoints.pop(0)

        self.target_position[0] = waypoint[0]
        self.target_position[1] = waypoint[1]
        self.target_position[2] = waypoint[2]
        print(f"Target position {self.target_position}.")

        self.cmd_position(*self.target_position, DEFAULT_HEADING_IN_RADIANS)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """Land the quadrotor.
        
        1. Command the drone to land.
        2. Transition to the LANDING state.
        """
        print("Landing transition.")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """Disarm the quadrotor.
        
        1. Command the drone to disarm.
        2. Transition to the DISARMING state.
        """
        print("Disarm transition.")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """Transition to manual mode.
        
        1. Release control of the drone.
        2. Stop the connection (and telemetry log).
        3. End the mission.
        4. Transition to the MANUAL state.
        """
        print("Manual transition.")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    @staticmethod
    def plot_altitude(time_vector, altitude_vector):
        """Plot altitude by time.

        :param time_vector: Time relative to start time.
        :param altitude_vector: Altitude relative to home position.
        """
        plt.plot(time_vector, altitude_vector)
        plt.xlabel("Time")
        plt.ylabel("Altitude")
        plt.title("Altitude by time")
        plt.show()

    @staticmethod
    def plot_position(north_vector, east_vector):
        """Plot coordinates.

        :param north_vector: North position, ordered by time.
        :param east_vector: East position, ordered by time.
        """
        plt.plot(east_vector, north_vector)
        plt.xlabel("East")
        plt.ylabel("North")
        plt.title("Square movement")
        plt.show()

    def start(self):
        """Start the quadrotor  simulation.
        
        1. Open a log file.
        2. Start the drone connection.
        3. Close the log file.
        """
        print("Creating log file.")
        self.start_log("Logs", "NavLog.txt")
        print("Starting connection.")
        self.connection.start()
        print("Closing log file.")
        self.stop_log()

        # Plot altitude.
        t_log = drone.read_telemetry_data("Logs/TLog.txt")
        local_data = t_log["MsgID.LOCAL_POSITION"]
        time_vector = local_data[0]
        altitude_vector = local_data[3] * DOWN_TO_ALTITUDE_CONVERSION
        self.plot_altitude(time_vector, altitude_vector)

        # Plot position.
        north_vector = local_data[1]
        east_vector = local_data[2]
        self.plot_position(north_vector, east_vector)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5760, help="Port number")
    parser.add_argument(
        "--host",
        type=str,
        default="127.0.0.1",
        help="host address, i.e. '127.0.0.1'",
    )
    args = parser.parse_args()

    conn = MavlinkConnection(
        "tcp:{0}:{1}".format(args.host, args.port), threaded=False, PX4=False
    )
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
