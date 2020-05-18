from queue import PriorityQueue
from typing import Callable, List, Tuple

import networkx as nx
import numpy as np
from bresenham import bresenham
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree


# Configuration.
NUMBER_OF_NEIGHBORS = 10
NUMBER_OF_SAMPLES = 10000


def extract_polygons(data: np.ndarray, safety_distance: int) -> List[Polygon]:
    """Extract polygons from city data.
    
    :param data: 2D numpy array with summarized obstacle data.
    :param safety_distance: Distance to keep from building.
    :return: List of polygons representing the city buildings.
    """

    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        point1 = (
            north - d_north - safety_distance,
            east - d_east - safety_distance,
        )
        point2 = (
            north + d_north + safety_distance,
            east - d_east - safety_distance,
        )
        point3 = (
            north + d_north + safety_distance,
            east + d_east + safety_distance,
        )
        point4 = (
            north - d_north - safety_distance,
            east + d_east + safety_distance,
        )
        corners = [point1, point2, point3, point4]
        poly = Polygon(corners)

        height = alt + d_alt
        polygons.append((poly, height))

    return polygons


def collides(
    polygons: List[Polygon], tree: KDTree, point: Tuple[int, int, int]
) -> bool:
    """Determine if a given point colides with given polygons.
    
    :param polygons: Polygons representing the city buildings.
    :param tree: KDTree containing the building centers.
    :param point: Coordinate to be verified for collision.
    :return: Boolean indicating True if collision, else False.
    """

    idxs = tree.query([point[0:2]], k=1, return_distance=False)[0][0]
    if polygons[idxs][0].contains(Point(point)) and polygons[idxs][1] >= point[2]:
        return True
    return False


def can_connect(
    polygons: List[Polygon], point1: Tuple[int, int, int], point2: Tuple[int, int, int],
):
    """Test for collision of an edge between two points and a polygon.

    The idea is that these points are candidates for our network nodes,
    however, the edge between them cannot cut any building.

    :param polygons: Polygons representing the city buildings.
    :param point1: First candidate node.
    :param point2: Second candidate node.
    :return: Boolean indicating True if edge lies in freespace, else False.
    """

    line = LineString([point1, point2])
    for polygon in polygons:
        if polygon[0].crosses(line) and polygon[1] >= min(point1[2], point2[2]):
            return False
    return True


def create_graph(
    data: np.ndarray, drone_altitude: int, safety_distance: int
) -> Tuple[np.ndarray, int, int]:
    """Returns a graph representation of a 2D configuration space.
    
    This is based on given obstacle data, minimum drone altitude and 
    safety distance arguments.

    :param data: 2D numpy array with summarized obstacle data.
    :param drone_altitude: Minimum height at which the drone should fly.
    :param safety_distance: Distance to keep from building.
    :return: Grid representation of obstacles, and center offsets.
    """

    # Find minimum and maximum north coordinates.
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # Find minimum and maximum east coordinates.
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    north_samples = np.random.randint(north_min, north_max, NUMBER_OF_SAMPLES)
    east_samples = np.random.randint(east_min, east_max, NUMBER_OF_SAMPLES)
    altitude_samples = [drone_altitude] * NUMBER_OF_SAMPLES
    samples = set(zip(north_samples, east_samples, altitude_samples))

    # Map city buildings into polygons.
    polygons = extract_polygons(data, safety_distance)
    centers = np.array(
        [(polygon[0].centroid.x, polygon[0].centroid.y) for polygon in polygons]
    )
    tree = KDTree(centers)

    nodes = []
    for point in samples:
        if not collides(polygons, tree, point):
            nodes.append(point)
    print("Number of non-colliding nodes: {}.".format(len(nodes)))

    city_graph = nx.Graph()
    tree = KDTree(nodes)
    for node in nodes:
        # Test for connectivity between each node and some of its neighbors.
        idxs = tree.query([node], NUMBER_OF_NEIGHBORS, return_distance=False)[0]
        # Iterate through all candidate nodes.
        for id in idxs:
            # If nodes are connectable, add an edge to the graph.
            if can_connect(polygons, node, nodes[id]):
                city_graph.add_edge(node, nodes[id])
    print("Graph creation completed.")

    return city_graph, int(north_min), int(east_min)


def find_nearest_node(
    graph: nx.Graph, position: Tuple[float, float, float]
) -> Tuple[int, int, int]:
    """Within a graph, find the nearest node to given position.

    :param graph: Graph of edges and nodes representing paths in freespace.
    :param position: Coordinates of position outside of graph.
    :return: Coordinates of node within graph.
    """
    tree = KDTree(graph.nodes)
    # Get nearest node.
    idxs = tree.query([position], k=1, return_distance=False)[0][0]
    return list(graph.nodes)[idxs]


def a_star(
    graph: np.ndarray,
    heuristic: Callable,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> Tuple[List[Tuple[int, int]], int]:
    """Find the lowest cost path from start to goal using A*.
    
    :param graph: Graph that defines paths in freespace.
    :param heuristic: Heuristic function to include directional knowledge.
    :param start: Start position.
    :param goal: Goal position.
    :return: Pruned list of coordinates from start to goal, and its cost.
    """

    # Find node nearest to both start and goal position.
    start_node = find_nearest_node(graph, start)
    print("Start position ({}) mapped to start node: {}.".format(start, start_node))
    goal_node = find_nearest_node(graph, goal)
    print("Goal position ({}) mapped to goal node: {}.".format(goal, goal_node))

    # Initialize data structures.
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start_node))
    visited = set(start_node)
    branch = {}
    found = False

    # Search the option space.
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start_node:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal_node:
            print("Found a path.")
            found = True
            break
        else:
            for next_node in nx.all_neighbors(graph, current_node):
                branch_cost = current_cost + np.linalg.norm(
                    np.array(next_node) - np.array(current_node)
                )
                queue_cost = branch_cost + heuristic(next_node, goal_node)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))

    if found:
        # Retrace steps.
        n = goal_node
        path_cost = branch[n][0]
        path.append(goal_node)
        while branch[n][1] != start_node:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print("**********************")
        print("Failed to find a path!")
        print("**********************")
    return path[::-1], path_cost


def heuristic(position: Tuple[int, int], goal_position: Tuple[int, int]) -> float:
    """Heuristic function to give directional sense to A*.
    
    :param position: Current position.
    :param goal_position: Position to navigate towards.
    :return: Cost estimate for moving from position to goal position.
    """

    return np.linalg.norm(np.array(position) - np.array(goal_position))


def prune_path(path: List[Tuple[int, int, int]]) -> List[Tuple[int, int, int]]:
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
