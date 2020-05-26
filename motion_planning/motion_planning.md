# 3D Motion Planning

## Files

Refer to `motion_planning.py` and `planning_utils.py` for all the below, except the final "extra" section.

## Explain the Starter Code

### 1. Explain the functionality of what is provided in `motion_planning.py` and `planning_utils.py`

The entry point to the `motion_planning.py` script is a bit more elaborate than the one for `backyard_flyer_solution.py`:
- The former accepts arguments (host and port) for configuring the simulator connection.
- More importantly, both create an object that inherited from Drone, initialize it with the simulator connection, and call upon `drone.start()`.

The start code is similar, it starts the event loop. There are a few differences regarding states, callbacks and handlers though:
- `PLANNING` is a new state: When the Drone is in this state, the `state_callback` will trigger `self.takeoff_transition()`. Previously this transition took place in the `ARMING` state.
- `ARMING` now triggers the path planning, which substitutes the Backyard Flyer its `self.calculate_box()` functionality for setting waypoints.
- Setting the home position is not part of arming anymore, this method is not explicitly called anymore.
- The altitude used to be set by the takeoff handler and the heading by the waypoint transition handler, now both are configured by the `plan_path()` method.

The biggest addition is the `plan_path()` method:
- It is triggered upon arming, and responsible for finding a feasible path and turning it into a set of waypoints.
- It reads the obstacle data and create a grid. Prior there was no notion of grid.
- A* is used to find a path from start to goal, this path i translated into waypoint.

The  implementation of the `create_grid()` and `a_star()` used in `plan_path()` can be found in the module `planning_utils.py`:
- Grid creation is by looping over all the provided data points and filling a three dimensional numpy array with obstacles.
- The A* implementation consequently is grid-based, currently allowing for moving in North, East, South or West direction from the current node.

## Implementing Your Path Planning Algorithm

### 1. Set your global home position

Here we read the first line of `colliders.csv` using a CSV reader, and parse the resulting two strings (one for latitude, other for longitude).
The drone API's `set_home_position()` is used to set the home position.

### 2. Set your current local position

Here we can rely on a utility function offered by the module `udacidrone.frame_utils` named `global_to_local()`. It accepts two arguments: global position and global home, and returns the local position.

### 3. Set grid start position from local position

We offset the local position with the grid's north and east offset, and pass the resulting coordinates to our A* implementation of the start position.

### 4. Set grid goal position from geodetic coordinates

Here we use the earlier mentioned `global_to_local()` helper function to transform geodetic coordinates into local coordinates. We offset these as well given our grid representation, and pass to A* as our goal.

### 5. Modify A* to include diagonal motion (or replace A* altogether)

We need to validate additional actions where we both go up and right, or down and left... in the grid. If these new positions do not contain an obstacle, then the action is valid. These new diagonal actions, including their delta and cost, can now be used by A*.

### 6. Cull waypoints 

Here we use the Besenham algorithm from the besenham Python package.
It works by looking at all waypoints except start and goal. If a waypoint exists that is on a Bresenham line with the earlier and later waypoint, then it is removed.
  
## Extra Challenges: Real World Planning

A probabilistic implementation is available too. Given that the simulator is slow, a "pre-generated" graph can be loaded from file. As graphs are more efficient, we can now traverse complex paths throughout the map.
