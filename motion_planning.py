import argparse
import csv
import time
from enum import Enum, auto

import msgpack
import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


# Configuration.
ALTITUDE_ERROR = 0.05
DEFAULT_HEADING_IN_RADIANS = 0.0
DOWN_TO_ALTITUDE_CONVERSION = -1.0
GLOBAL_DISARMING_ALTITUDE = 0.1
# GOAL_LATITUDE = 37.79681984
# GOAL_LONGITUDE = -122.40010668
GROUND_LEVEL_ALTITUDE = 0.0
LOCAL_DISARMING_ALTITUDE = 0.01
MAXIMUM_LANDING_VELOCITY = 1.0
POSITION_ERROR_IN_METERS = 1.0
SAFETY_DISTANCE = 7.0
TARGET_ALTITUDE_IN_METERS = 5


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):
    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # Initial state.
        self.flight_state = States.MANUAL

        # Register all callbacks here.
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if (
                DOWN_TO_ALTITUDE_CONVERSION * self.local_position[2]
                > (1 - ALTITUDE_ERROR) * self.target_position[2]
            ):
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if (
                np.linalg.norm(self.target_position[0:2] - self.local_position[0:2])
                < POSITION_ERROR_IN_METERS
            ):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if (
                        np.linalg.norm(self.local_velocity[0:2])
                        < MAXIMUM_LANDING_VELOCITY
                    ):
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if (
                self.global_position[2] - self.global_home[2]
                < GLOBAL_DISARMING_ALTITUDE
                and abs(self.local_position[2]) < LOCAL_DISARMING_ALTITUDE
            ):
                self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("Arming transition.")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("Takeoff transition: {}.".format(self.target_position[2]))
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("Waypoint transition.")
        self.target_position = self.waypoints.pop(0)
        print("Target position.", self.target_position)
        self.cmd_position(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            self.target_position[3],
        )

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("Landing transition.")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("Disarm transition.")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("Manual transition.")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path...")
        self.target_position[2] = TARGET_ALTITUDE_IN_METERS

        # Read home coordinates.
        with open("colliders.csv") as file:
            reader = csv.reader(file, delimiter=",")
            lat0, lon0 = next(reader)

        # Set home position.
        lat0 = float(lat0.strip().split(" ")[1])
        lon0 = float(lon0.strip().split(" ")[1])
        self.set_home_position(lon0, lat0, GROUND_LEVEL_ALTITUDE)

        # Set local position.
        local_position = global_to_local(self.global_position, self.global_home,)
        print(
            "Global home {0}, position {1}, local position {2}.".format(
                self.global_home, self.global_position, self.local_position
            )
        )

        # Read in obstacle map.
        data = np.loadtxt("colliders.csv", delimiter=",", dtype="Float64", skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles.
        grid, north_offset, east_offset = create_grid(
            data, TARGET_ALTITUDE_IN_METERS, SAFETY_DISTANCE
        )
        print("North offset = {0}, east offset = {1}.".format(north_offset, east_offset))

        # Define starting point on the grid, set to local position.
        grid_start = (
            -north_offset + int(local_position[0]),
            -east_offset + int(local_position[1]),
        )

        # Set goal as some arbitrary position on the grid.
        goal_location = global_to_local(
            (GOAL_LONGITUDE, GOAL_LATITUDE, GROUND_LEVEL_ALTITUDE),
            self.global_home
        )
        grid_goal = (
            -north_offset + int(goal_location[0]), 
            -east_offset + int(goal_location[1])
        )

        # Run A* to find a path from start to goal.
        print("Local Start and Goal: {}, {}.".format(grid_start, grid_goal))
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        pruned_path = prune_path(path)

        # Convert path to waypoints.
        waypoints = [
            [
                p[0] + north_offset,
                p[1] + east_offset,
                TARGET_ALTITUDE_IN_METERS,
                DEFAULT_HEADING_IN_RADIANS,
            ]
            for p in pruned_path
        ]
        # Set self.waypoints and send to simulator for visualization.
        self.waypoints = waypoints
        print(self.waypoints)
        
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("Starting connection.")
        self.connection.start()
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5760, help="Port number")
    parser.add_argument(
        "--host", type=str, default="127.0.0.1", help="host address, i.e. '127.0.0.1'",
    )
    parser.add_argument(
        "--longitude", type=float, default="-122.396", help="Goal longitude",
    )
    parser.add_argument(
        "--latitude", type=float, default="37.794", help="Goal latitude",
    )
    args = parser.parse_args()

    # Read arguments and set as configuration.
    GOAL_LATITUDE = args.latitude
    GOAL_LONGITUDE = args.longitude

    conn = MavlinkConnection("tcp:{0}:{1}".format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
