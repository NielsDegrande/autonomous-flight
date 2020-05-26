from enum import Enum
from queue import PriorityQueue
from typing import Callable, List, Tuple

import numpy as np
from bresenham import bresenham


# Configuration.
DIAGONAL_MOVEMENT_COST = np.sqrt(2)


def create_grid(
    data: np.ndarray, drone_altitude: int, safety_distance: int
) -> Tuple[np.ndarray, int, int]:
    """Returns a grid representation of a 2D configuration space.
    
    This is based on given obstacle data, drone altitude and 
    safety distance arguments.

    :param data: 2D numpy array with summarized obstacle data.
    :param drone_altitude: Height at which the drone should fly.
    :param safety_distance: Distance to keep from building.
    :return: Grid representation of obstacles, and center offsets.
    """

    # Find minimum and maximum north coordinates.
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # Find minimum and maximum east coordinates.
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # Calculate the size of the grid given the minima and maxima.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid.
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles.
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(
                    np.clip(
                        north - d_north - safety_distance - north_min,
                        0,
                        north_size - 1,
                    )
                ),
                int(
                    np.clip(
                        north + d_north + safety_distance - north_min,
                        0,
                        north_size - 1,
                    )
                ),
                int(
                    np.clip(
                        east - d_east - safety_distance - east_min,
                        0,
                        east_size - 1,
                    )
                ),
                int(
                    np.clip(
                        east + d_east + safety_distance - east_min,
                        0,
                        east_size - 1,
                    )
                ),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    NORTH = (-1, 0, 1)
    NORTH_EAST = (-1, 1, DIAGONAL_MOVEMENT_COST)
    EAST = (0, 1, 1)
    SOUTH_EAST = (1, 1, DIAGONAL_MOVEMENT_COST)
    SOUTH = (1, 0, 1)
    SOUTH_WEST = (1, -1, DIAGONAL_MOVEMENT_COST)
    WEST = (0, -1, 1)
    NORTH_WEST = (-1, -1, DIAGONAL_MOVEMENT_COST)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid: np.ndarray, current_node: Tuple[int, int]) -> List[Action]:
    """Returns a list of valid actions given a grid and current node."""

    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # Check if the node is off the grid or if it is an obstacle.
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)

    return valid_actions


def a_star(
    grid: np.ndarray,
    h: Callable,
    start: Tuple[int, int],
    goal: Tuple[int, int]
) -> Tuple[List[Tuple[int, int]], int]:
    """Find the lowest cost path from start to goal using A*.
    
    :param grid: Grid that defines obstacles.
    :param h: Heuristic function to include directional knowledge.
    :param start: Start position.
    :param goal: Goal position.
    :return: Pruned list of coordinates from start to goal, and its cost.
    """

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print("Found a path.")
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # Get the tuple representation.
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # Retrace steps.
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print("**********************")
        print("Failed to find a path!")
        print("**********************")
    # Return path (in reversed order) and cost.
    return path[::-1], path_cost


def heuristic(position: Tuple[int, int], goal_position: Tuple[int, int]) -> float:
    """Heuristic function to give directional sense to A*.
    
    :param position: Current position.
    :param goal_position: Position to navigate towards.
    :return: Cost estimate for moving from position to goal position.
    """
    
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def prune_path(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """Prune path of nodes using the Bresenham algorithm.
    
    :param path: A fully connected path.
    :return: A pruned path, removing unnecessary points.
    """

    pruned_path = []
    for i, p in enumerate(path):
        if (i == 0) or (i == (len(path) - 1)):
            pruned_path.append(p)
        else:
            cells = list(
                bresenham(
                    path[i - 1][0], path[i - 1][1], path[i + 1][0], path[i + 1][1],
                )
            )
            if p not in cells:
                pruned_path.append(p)

    print(
        "Length of path: {}, length of pruned path: {}.".format(
            len(path), len(pruned_path)
        )
    )
    return pruned_path
