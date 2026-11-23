"""Heading-aware A* initializer for the three-circle quadruped footprint.

This is a geometric warm start, NOT a dynamics/turn-rate feasible path.
Each path-tangent edge uses one of the eight grid headings; in-place heading
changes are checked at nodes before taking the next edge. The collocation NLP
subsequently verifies the actual footprint at its own nominal yaw samples.
"""

import heapq
from collections import deque
import numpy as np

from planning.planner_helpers import polygon_halfspaces
from planning.robot_footprint import three_circle_parameters


# Every edge's heading is tied to its grid movement direction.
_DIRECTIONS = ((1, 0), (1, 1), (0, 1), (-1, 1),
               (-1, 0), (-1, -1), (0, -1), (1, -1))
_HEADINGS = np.arctan2([dy for _, dy in _DIRECTIONS],
                       [dx for dx, _ in _DIRECTIONS])


def _segment_hits_convex_polygon(p0, p1, normals, offsets):
    """Exact segment intersection with convex halfspaces n @ p <= b.

    The obstacle is already conservatively inflated by the circle radius.
    Intersect [0,1] with the interval permitted by every halfspace.
    """
    direction = p1 - p0
    t_min, t_max = 0.0, 1.0
    for normal, bound in zip(normals, offsets):
        numerator = bound - normal @ p0
        denominator = normal @ direction
        if abs(denominator) < 1e-12:
            if numerator < 0.0:  # Segment lies outside this face.
                return False
        elif denominator > 0.0:
            t_max = min(t_max, numerator / denominator)
        else:
            t_min = max(t_min, numerator / denominator)
        if t_min > t_max:
            return False
    return t_min <= t_max and t_max >= 0.0 and t_min <= 1.0


def astar_three_circle_path(
    start_state, goal_state, bounds, obstacles, length, width,
    safety_margin=0.05, resolution=0.05, extra_clearance=0.01,
    yaw_mode="path_tangent", fixed_yaw=0.0, turn_penalty=0.05,
):
    """Find a collision-checked geometric A* path for three body circles.

    `yaw_mode='path_tangent'`: headings are eight grid directions, and a
    change of direction is checked as a rotation at the intermediate node.
    `yaw_mode='fixed'`: the body maintains `fixed_yaw` while translating.

    The returned polyline is an INITIAL GUESS. The smooth yaw reference used
    later in collocation may differ from these discrete A* orientations;
    this search is not a certificate of the final optimized trajectory.
    """
    values = np.asarray([*bounds, length, width, safety_margin,
                         resolution, extra_clearance, fixed_yaw, turn_penalty], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("A* parameters must be finite")
    x_min, x_max, y_min, y_max = bounds
    if x_min >= x_max or y_min >= y_max or resolution <= 0:
        raise ValueError("Invalid A* bounds or resolution")
    if safety_margin < 0 or extra_clearance < 0 or turn_penalty < 0:
        raise ValueError("Margins and turn_penalty must be nonnegative")
    if yaw_mode not in ("path_tangent", "fixed"):
        raise ValueError("yaw_mode must be 'path_tangent' or 'fixed'")

    start = np.asarray(start_state[:2], dtype=float)
    goal = np.asarray(goal_state[:2], dtype=float)
    if not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
        raise ValueError("Start and goal must be finite")

    offsets, radius = three_circle_parameters(length, width,
                                               safety_margin + extra_clearance)
    inflated_polygons = [polygon_halfspaces(np.asarray(obs, dtype=float), radius)
                         for obs in obstacles]
    nx = int(np.floor((x_max - x_min) / resolution)) + 1
    ny = int(np.floor((y_max - y_min) / resolution)) + 1

    def position(index):
        return np.array([x_min + index[0] * resolution,
                         y_min + index[1] * resolution])

    def index_of(p):
        return (int(round((p[0] - x_min) / resolution)),
                int(round((p[1] - y_min) / resolution)))

    def in_grid(ij):
        return 0 <= ij[0] < nx and 0 <= ij[1] < ny

    def pose_safe(p, yaw):
        # Circle centers = CoM + longitudinal offsets rotated by yaw.
        along = np.array([np.cos(yaw), np.sin(yaw)])
        centers = p[None, :] + offsets[:, None] * along
        if np.any(centers[:, 0] <= x_min + radius) or np.any(centers[:, 0] >= x_max - radius):
            return False
        if np.any(centers[:, 1] <= y_min + radius) or np.any(centers[:, 1] >= y_max - radius):
            return False
        for center in centers:
            for normals, bounds_poly in inflated_polygons:
                if np.all(normals @ center <= bounds_poly):
                    return False
        return True

    def segment_safe(p0, p1, yaw):
        along = np.array([np.cos(yaw), np.sin(yaw)])
        for offset in offsets:
            c0 = p0 + offset * along
            c1 = p1 + offset * along
            # Endpoints imply wall safety throughout the straight segment.
            if not (x_min + radius < c0[0] < x_max - radius and
                    y_min + radius < c0[1] < y_max - radius and
                    x_min + radius < c1[0] < x_max - radius and
                    y_min + radius < c1[1] < y_max - radius):
                return False
            for normals, poly_offsets in inflated_polygons:
                if _segment_hits_convex_polygon(c0, c1, normals, poly_offsets):
                    return False
        return True

    # Precompute turn-sweep offsets for the 8 heading pairs. Collision
    # tests then check all 3 circles at all intermediate yaw samples at once.
    turn_offsets = {}
    if yaw_mode == "path_tangent":
        for previous in range(8):
            for upcoming in range(8):
                if previous == upcoming:
                    continue
                delta = np.arctan2(np.sin(_HEADINGS[upcoming] - _HEADINGS[previous]),
                                   np.cos(_HEADINGS[upcoming] - _HEADINGS[previous]))
                steps = max(1, int(np.ceil(abs(delta) / np.deg2rad(5.0))))
                angles = _HEADINGS[previous] + delta * np.linspace(0.0, 1.0, steps + 1)
                dirs = np.column_stack((np.cos(angles), np.sin(angles)))
                turn_offsets[previous, upcoming] = (dirs[:, None, :] *
                                                     offsets[None, :, None]).reshape(-1, 2)
    turn_cache = {}

    def turn_safe(ij, previous, upcoming):
        if yaw_mode == "fixed" or previous == upcoming:
            return True
        key = (*ij, previous, upcoming)
        if key not in turn_cache:
            centers = position(ij) + turn_offsets[previous, upcoming]
            safe = bool(np.all((centers[:, 0] > x_min + radius) &
                               (centers[:, 0] < x_max - radius) &
                               (centers[:, 1] > y_min + radius) &
                               (centers[:, 1] < y_max - radius)))
            if safe:
                for normals, poly_offsets in inflated_polygons:
                    if np.any(np.all(normals @ centers.T <= poly_offsets[:, None], axis=0)):
                        safe = False
                        break
            turn_cache[key] = safe
        return turn_cache[key]

    start_index = index_of(start)
    goal_index = index_of(goal)
    if not in_grid(start_index) or not in_grid(goal_index):
        raise ValueError("A* start or goal is outside the course bounds")

    # Necessary-condition precheck: the center circle must fit at every
    # visited grid cell, irrespective of yaw. A flood fill from the goal
    # rules out disconnected passages cheaply before heading-aware search.
    # Because this RELAXES heading and edge checks, it cannot rule out a
    # path that the heading-aware grid search would accept.
    free_center_cache = {}

    def center_free(ij):
        if ij not in free_center_cache:
            if not in_grid(ij):
                return False
            p = position(ij)
            safe = (x_min + radius < p[0] < x_max - radius and
                    y_min + radius < p[1] < y_max - radius)
            if safe:
                safe = all(not np.all(n @ p <= b)
                           for n, b in inflated_polygons)
            free_center_cache[ij] = safe
        return free_center_cache[ij]

    reachable = {goal_index} if center_free(goal_index) else set()
    queue = deque(reachable)
    while queue:
        i, j = queue.popleft()
        for di, dj in _DIRECTIONS:
            ij = (i + di, j + dj)
            if ij not in reachable and center_free(ij):
                reachable.add(ij)
                queue.append(ij)
    if start_index not in reachable:
        if not center_free(start_index):
            reason = "start center is blocked at its nearest grid node"
        elif not center_free(goal_index):
            reason = "goal center is blocked at its nearest grid node"
        else:
            reason = "center-circle grid connectivity search found no route"
        raise RuntimeError(
            f"Three-circle A* initialization failed ({reason}; "
            f"resolution={resolution:g} m). A grid-search failure does not "
            "establish that no continuous collision-free path exists."
        )

    # The initial yaw is unconstrained by the four-state planner; choose
    # the heading of the first travel edge. In fixed mode use fixed_yaw.
    start_node = (*start_index, -1)
    g_score = {start_node: 0.0}
    came_from = {}
    heap = [(float(np.linalg.norm(goal - start)), 0.0, start_node)]
    closed = set()
    pose_cache = {}
    edge_cache = {}

    def valid_pose(ij, h):
        key = (*ij, h)
        if key not in pose_cache:
            pose_cache[key] = in_grid(ij) and pose_safe(
                position(ij), fixed_yaw if yaw_mode == "fixed" else _HEADINGS[h])
        return pose_cache[key]

    success = None
    while heap:
        _, cost, node = heapq.heappop(heap)
        if node in closed or cost > g_score[node] + 1e-10:
            continue
        closed.add(node)
        i, j, previous_heading = node
        at = position((i, j))
        if (i, j) == goal_index and previous_heading != -1:
            final_yaw = fixed_yaw if yaw_mode == "fixed" else _HEADINGS[previous_heading]
            if segment_safe(at, goal, final_yaw):
                success = node
                break

        for heading_idx, (di, dj) in enumerate(_DIRECTIONS):
            neighbor = (i + di, j + dj)
            next_heading = 0 if yaw_mode == "fixed" else heading_idx
            if neighbor not in reachable or not valid_pose(neighbor, next_heading):
                continue
            yaw = fixed_yaw if yaw_mode == "fixed" else _HEADINGS[heading_idx]
            if previous_heading != -1 and yaw_mode == "path_tangent":
                if not turn_safe((i, j), previous_heading, heading_idx):
                    continue
            elif previous_heading == -1:
                if not pose_safe(start, yaw) or not segment_safe(start, at, yaw):
                    continue

            edge_key = (i, j, heading_idx)  # Same edge for any incoming yaw.
            if edge_key not in edge_cache:
                edge_cache[edge_key] = segment_safe(at, position(neighbor), yaw)
            if not edge_cache[edge_key]:
                continue

            angle_change = (0.0 if previous_heading < 0 or yaw_mode == "fixed"
                            else abs(np.arctan2(np.sin(yaw - _HEADINGS[previous_heading]),
                                              np.cos(yaw - _HEADINGS[previous_heading]))))
            new_cost = cost + resolution * np.hypot(di, dj) + turn_penalty * angle_change
            new_node = (*neighbor, next_heading)
            if new_cost < g_score.get(new_node, np.inf):
                g_score[new_node] = new_cost
                came_from[new_node] = node
                priority = new_cost + float(np.linalg.norm(position(neighbor) - goal))
                heapq.heappush(heap, (priority, new_cost, new_node))

    if success is None:
        raise RuntimeError(
            "Three-circle A* initialization failed (heading/edge/turn "
            f"search exhausted; resolution={resolution:g} m). "
            "A grid-search failure does not establish that no continuous "
            "collision-free path exists."
        )

    nodes = [success]
    while nodes[-1] != start_node:
        nodes.append(came_from[nodes[-1]])
    nodes.reverse()
    polyline = np.array([position((i, j)) for i, j, _ in nodes], dtype=float)
    polyline[0] = start
    polyline[-1] = goal
    return polyline
