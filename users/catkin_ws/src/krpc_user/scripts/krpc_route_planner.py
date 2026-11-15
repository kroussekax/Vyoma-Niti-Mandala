# -*- coding:utf-8 -*-
"""
krpc_route_planner.py - choose the fastest order to visit the checkpoints.
==========================================================================

The problem
    Start where the robot is now, visit every checkpoint once, and end at the
    goal checkpoint. Which order is fastest? This is the "Travelling Salesman
    Problem" (TSP).

Why not try every order?
    With 11 checkpoints there are 11! = 39,916,800 orders, which is too slow.

What we use: Held-Karp (dynamic programming)
    Remember the best time for each "state" instead of each full order. A state is:
        (set of checkpoints already visited, checkpoint where we are now)
    Example: "visited {3, 5, 7}, now at 7" has one best time. Every order that
    reaches this state can only continue the same way, so we keep only the best.
    There are 2^11 * 11 = 22,528 states, which is fast, and the answer is still
    the EXACT optimum. A set is stored as a bitmask: bit k = 1 means "node k is visited".
"""

import math

import krpc_config as cfg
import krpc_geometry as geo


def leg_time(from_pos, from_quat, to_pos, to_quat,
             v_est=None, w_est=None, overhead=None):
    """Estimated seconds to fly from one pose to another.

    The robot moves and turns at the same time, so the slower of the two wins:
        max(distance / V_EST, rotation / W_EST) + LEG_OVERHEAD
    """
    v_est = cfg.V_EST if v_est is None else v_est
    w_est = cfg.W_EST if w_est is None else w_est
    overhead = cfg.LEG_OVERHEAD if overhead is None else overhead
    translation = geo.distance(from_pos, to_pos) / v_est
    rotation = math.radians(geo.quat_angle_deg(from_quat, to_quat)) / w_est
    return max(translation, rotation) + overhead


def plan_route(start_pose, nodes, goal_node=None, cost_fn=leg_time):
    """Find the fastest visiting order.

    start_pose  (position, quaternion) of the robot now
    nodes       [(id, position, quaternion), ...] to visit in ANY order
    goal_node   (id, position, quaternion) that must be LAST, or None

    Returns (list of ids in visiting order, estimated total seconds).
    """
    n = len(nodes)
    if n == 0:
        if goal_node is None:
            return [], 0.0
        return [goal_node[0]], cost_fn(start_pose[0], start_pose[1], goal_node[1], goal_node[2])

    # Pre-compute every flight time once (faster than recomputing inside the loops).
    start_cost = [cost_fn(start_pose[0], start_pose[1], nd[1], nd[2]) for nd in nodes]
    pair_cost = [[0.0 if i == j else cost_fn(a[1], a[2], b[1], b[2])
                  for j, b in enumerate(nodes)] for i, a in enumerate(nodes)]
    end_cost = [cost_fn(nd[1], nd[2], goal_node[1], goal_node[2]) if goal_node else 0.0
                for nd in nodes]

    full = (1 << n) - 1                     # bitmask with all n nodes visited
    inf = float('inf')
    # best[mask][j]   = smallest time to visit exactly the nodes in `mask`, ending at node j
    # parent[mask][j] = the node visited just before j on that best path (to rebuild the order)
    best = [[inf] * n for _ in range(1 << n)]
    parent = [[-1] * n for _ in range(1 << n)]
    for j in range(n):
        best[1 << j][j] = start_cost[j]     # first flight: start -> node j

    # Grow every path by one more node at a time.
    for mask in range(1, full + 1):
        row = best[mask]
        for j in range(n):
            base = row[j]
            if base == inf:                 # this state cannot be reached
                continue
            costs_j = pair_cost[j]
            for k in range(n):
                bit = 1 << k
                if mask & bit:              # k is already visited
                    continue
                candidate = base + costs_j[k]
                nxt = mask | bit
                if candidate < best[nxt][k]:
                    best[nxt][k] = candidate
                    parent[nxt][k] = j

    # All nodes visited: choose the last node that makes "... -> goal" cheapest.
    last = min(range(n), key=lambda j: best[full][j] + end_cost[j])
    total = best[full][last] + end_cost[last]

    # Walk the parent links backwards to rebuild the order.
    order = []
    mask, j = full, last
    while j != -1:
        order.append(nodes[j][0])
        prev = parent[mask][j]
        mask &= ~(1 << j)
        j = prev
    order.reverse()
    if goal_node is not None:
        order.append(goal_node[0])
    return order, total
