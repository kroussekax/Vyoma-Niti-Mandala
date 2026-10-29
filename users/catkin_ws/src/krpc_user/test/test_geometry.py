# -*- coding:utf-8 -*-
"""Tests for krpc_geometry.py: our 3D math must give the same numbers as the
simulator's judge.py and generate_objects.py."""
import math
import random
import unittest

import krpc_test_support as support
import krpc_geometry as geo


def random_quaternion(rng):
    return geo.quat_normalize([rng.gauss(0, 1) for _ in range(4)])


def random_unit_vector(rng):
    return geo.normalize_vector([rng.gauss(0, 1) for _ in range(3)])


class GeometryBasicsTest(unittest.TestCase):

    def test_rotate_identity_and_axis(self):
        self.assertEqual(geo.rotate_vector([1, 2, 3], [0, 0, 0, 1]), [1.0, 2.0, 3.0])
        q = geo.quat_from_axis_angle([0, 0, 1], math.pi / 2)
        v = geo.rotate_vector([1, 0, 0], q)
        for got, want in zip(v, [0, 1, 0]):
            self.assertAlmostEqual(got, want, places=12)

    def test_look_at_points_x_axis_forward(self):
        rng = random.Random(1)
        for _ in range(500):
            forward = random_unit_vector(rng)
            q = geo.look_at_quaternion(forward)
            self.assertLess(geo.vector_angle_deg(geo.rotate_vector([1, 0, 0], q), forward), 1e-6)

    def test_look_at_with_roll_keeps_twist_and_centers(self):
        rng = random.Random(2)
        for _ in range(500):
            position = [rng.uniform(0, 7), rng.uniform(-1, 1), rng.uniform(0, 2)]
            marker = geo.sub(position, geo.normalize_vector(random_unit_vector(rng)))
            reference = random_quaternion(rng)
            q = geo.look_at_with_roll(position, marker, reference)
            centering, roll = geo.centering_and_roll_deg(position, q, marker, reference)
            self.assertLess(centering, 1e-5)
            self.assertLess(roll, 1e-5)

    def test_quat_angle(self):
        q = geo.quat_from_axis_angle([0, 1, 0], math.radians(30))
        self.assertAlmostEqual(geo.quat_angle_deg([0, 0, 0, 1], q), 30.0, places=9)
        self.assertAlmostEqual(geo.quat_angle_deg(q, [-c for c in q]), 0.0, places=5)


@unittest.skipUnless(support.SIMULATOR_AVAILABLE, 'simulator sources (rootfs/) not found')
class GeometryMatchesSimulatorTest(unittest.TestCase):

    def setUp(self):
        self.referee = support.make_referee({'Checkpoints': []})
        self.gen = support.generator_module()
        self.rng = random.Random(3)

    def test_rotate_matches_judge(self):
        for _ in range(1000):
            q, v = random_quaternion(self.rng), random_unit_vector(self.rng)
            for got, want in zip(geo.rotate_vector(v, q), self.referee._rotate_vector_by_quaternion(v, q)):
                self.assertAlmostEqual(got, want, places=12)

    def test_twist_matches_judge(self):
        for _ in range(1000):
            q, axis = random_quaternion(self.rng), random_unit_vector(self.rng)
            self.assertAlmostEqual(geo.twist_angle_deg(q, axis), self.referee._twist_angle_deg(q, axis), places=9)

    def test_centering_and_roll_match_judge(self):
        for _ in range(1000):
            position = [self.rng.uniform(0, 7) for _ in range(3)]
            marker = [self.rng.uniform(0, 7) for _ in range(3)]
            q, target = random_quaternion(self.rng), random_quaternion(self.rng)
            ours = geo.centering_and_roll_deg(position, q, marker, target)
            theirs = self.referee._evaluate_centering_and_roll(position, q, marker, target)
            self.assertAlmostEqual(ours[0], theirs[0], places=7)
            self.assertAlmostEqual(ours[1], theirs[1], places=7)

    def test_multiply_and_look_at_match_generator(self):
        for _ in range(500):
            a, b = random_quaternion(self.rng), random_quaternion(self.rng)
            for got, want in zip(geo.quat_multiply(a, b), self.gen.quaternion_multiply(a, b)):
                self.assertAlmostEqual(got, want, places=12)
            forward = random_unit_vector(self.rng)
            self.assertLess(geo.quat_angle_deg(geo.look_at_quaternion(forward),
                                               self.gen.quaternion_from_forward_axis(forward)), 1e-5)


if __name__ == '__main__':
    unittest.main()
