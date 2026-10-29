# -*- coding:utf-8 -*-
"""Tests for krpc_route_planner.py: Held-Karp must find the same best total time
as trying every possible order (brute force) on small examples."""
import itertools
import random
import time
import unittest

import krpc_test_support  # noqa: F401  (adds scripts/ to sys.path)
import krpc_geometry as geo
import krpc_route_planner as planner


def random_node(rng, node_id):
    position = [rng.uniform(0.2, 7.0), rng.uniform(-0.8, 0.85), rng.uniform(0.0, 1.68)]
    quaternion = geo.quat_normalize([rng.gauss(0, 1) for _ in range(4)])
    return (node_id, position, quaternion)


def route_cost(start, route_nodes):
    total, pos, quat = 0.0, start[0], start[1]
    for _, p, q in route_nodes:
        total += planner.leg_time(pos, quat, p, q)
        pos, quat = p, q
    return total


def brute_force(start, nodes, goal):
    best = None
    for perm in itertools.permutations(nodes):
        seq = list(perm) + ([goal] if goal else [])
        cost = route_cost(start, seq)
        if best is None or cost < best:
            best = cost
    return best


class RoutePlannerTest(unittest.TestCase):

    def test_matches_brute_force(self):
        rng = random.Random(42)
        for trial in range(40):
            n = rng.randint(1, 7)
            nodes = [random_node(rng, i) for i in range(1, n + 1)]
            goal = random_node(rng, 12) if trial % 4 else None
            start = random_node(rng, 0)[1:]
            order, total = planner.plan_route(start, nodes, goal)

            by_id = dict((nd[0], nd) for nd in nodes + ([goal] if goal else []))
            self.assertEqual(sorted(order), sorted(by_id))
            if goal:
                self.assertEqual(order[-1], 12)
            self.assertAlmostEqual(total, route_cost(start, [by_id[i] for i in order]), places=9)
            self.assertAlmostEqual(total, brute_force(start, nodes, goal), places=9)

    def test_only_goal(self):
        rng = random.Random(1)
        goal = random_node(rng, 12)
        order, total = planner.plan_route(random_node(rng, 0)[1:], [], goal)
        self.assertEqual(order, [12])
        self.assertGreater(total, 0.0)

    def test_veryhard_size_is_fast(self):
        rng = random.Random(7)
        nodes = [random_node(rng, i) for i in range(1, 12)]
        started = time.time()
        order, _ = planner.plan_route(random_node(rng, 0)[1:], nodes, random_node(rng, 12))
        self.assertEqual(len(order), 12)
        self.assertLess(time.time() - started, 5.0)


if __name__ == '__main__':
    unittest.main()
