import numpy as np
import heapq


def polygon_halfspaces(polygon, inflation_radius):
    """Helper function to compute halfspaces of a polygon """
    normals = []
    offsets = []

    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]

        edge = p2 - p1

        # outward normal for counter-clockwise polygon
        normal = np.array([edge[1], -edge[0]], dtype=float)
        normal /= np.linalg.norm(normal)

        b = normal @ p1

        # move boundary outward by robot radius
        b += inflation_radius

        normals.append(normal)
        offsets.append(b)

    return np.array(normals), np.array(offsets)

def point_in_inflated_polygon(point, polygon, inflation_radius):
    """
    Returns True if point lies inside the inflated convex polygon.
    Uses the same halfspace representation as the NLP obstacle constraint.
    """
    normals, offsets = polygon_halfspaces(polygon, inflation_radius)

    point = np.asarray(point, dtype=float)
    s = normals @ point - offsets

    return np.all(s <= 0.0)


def astar_initial_path(
    start_state,
    goal_state,
    bounds,
    obstacles,
    inflation_radius,
    resolution=0.1,
    extra_clearance=0.01,
):
    """
    Run 8-connected A* on a 2-D grid to generate a geometric
    initial path for the direct-collocation NLP.
    """

    x_min, x_max, y_min, y_max = bounds

    start = np.asarray(start_state[:2], dtype=float)
    goal = np.asarray(goal_state[:2], dtype=float)

    nx = int(np.floor((x_max - x_min) / resolution)) + 1
    ny = int(np.floor((y_max - y_min) / resolution)) + 1

    def index_to_position(index):
        i, j = index
        return np.array([
            x_min + i * resolution,
            y_min + j * resolution,
        ])

    def position_to_index(position):
        i = int(round((position[0] - x_min) / resolution))
        j = int(round((position[1] - y_min) / resolution))
        return i, j

    def valid_index(index):
        i, j = index
        return 0 <= i < nx and 0 <= j < ny

    free_cache = {}

    def is_free(index):

        if index in free_cache:
            return free_cache[index]

        if not valid_index(index):
            return False

        position = index_to_position(index)

        for obstacle in obstacles:
            if point_in_inflated_polygon(
                position,
                obstacle,
                inflation_radius + extra_clearance,
            ):
                free_cache[index] = False
                return False

        free_cache[index] = True
        return True

    start_index = position_to_index(start)
    goal_index = position_to_index(goal)

    if not is_free(start_index):
        raise ValueError("A* start lies inside an inflated obstacle")

    if not is_free(goal_index):
        raise ValueError("A* goal lies inside an inflated obstacle")

    # 8-connected grid
    neighbor_offsets = [
        (-1, 0), (1, 0),
        (0, -1), (0, 1),
        (-1, -1), (-1, 1),
        (1, -1), (1, 1),
    ]

    def heuristic(index):
        di = index[0] - goal_index[0]
        dj = index[1] - goal_index[1]
        return resolution * np.hypot(di, dj)

    open_set = []

    heapq.heappush(
        open_set,
        (heuristic(start_index), 0.0, start_index)
    )

    came_from = {}
    g_score = {start_index: 0.0}
    closed = set()

    found = False

    while open_set:

        _, current_cost, current = heapq.heappop(open_set)

        if current in closed:
            continue

        if current == goal_index:
            found = True
            break

        closed.add(current)

        i, j = current

        for di, dj in neighbor_offsets:

            neighbor = (i + di, j + dj)

            if not is_free(neighbor):
                continue

            # Prevent diagonally cutting through obstacle corners
            if di != 0 and dj != 0:
                if (
                    not is_free((i + di, j))
                    or not is_free((i, j + dj))
                ):
                    continue

            step_cost = resolution * np.hypot(di, dj)
            tentative_cost = current_cost + step_cost

            if tentative_cost < g_score.get(neighbor, np.inf):

                came_from[neighbor] = current
                g_score[neighbor] = tentative_cost

                priority = tentative_cost + heuristic(neighbor)

                heapq.heappush(
                    open_set,
                    (priority, tentative_cost, neighbor)
                )

    if not found:
        raise RuntimeError(
            "A* could not find a collision-free path from start to goal"
        )

    # Reconstruct grid path
    path_indices = [goal_index]

    while path_indices[-1] != start_index:
        path_indices.append(came_from[path_indices[-1]])

    path_indices.reverse()

    path = np.array(
        [index_to_position(index) for index in path_indices]
    )

    # Preserve exact continuous start and goal
    path[0] = start
    path[-1] = goal

    return path


def sample_path(alpha, path):
    """
    Sample a point at fraction alpha along the A* path,
    based on cumulative path length.
    """

    path = np.asarray(path, dtype=float)

    if len(path) == 1:
        return path[0].copy()

    segments = path[1:] - path[:-1]
    segment_lengths = np.linalg.norm(segments, axis=1)

    cumulative_length = np.concatenate(
        ([0.0], np.cumsum(segment_lengths))
    )

    total_length = cumulative_length[-1]

    if total_length < 1e-12:
        return path[0].copy()

    target_length = np.clip(alpha, 0.0, 1.0) * total_length

    if target_length >= total_length:
        return path[-1].copy()

    segment_index = (
        np.searchsorted(
            cumulative_length,
            target_length,
            side="right",
        ) - 1
    )

    segment_index = min(
        max(segment_index, 0),
        len(segments) - 1,
    )

    local_length = (
        target_length
        - cumulative_length[segment_index]
    )

    if segment_lengths[segment_index] < 1e-12:
        return path[segment_index].copy()

    local_alpha = (
        local_length
        / segment_lengths[segment_index]
    )

    return (
        path[segment_index]
        + local_alpha * segments[segment_index]
    )