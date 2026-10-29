# -*- coding:utf-8 -*-
"""Tests for krpc_judge_replica.py: for thousands of random poses our level must
be exactly the level the real judge.py would give."""
import json
import math
import random
import unittest

import krpc_test_support as support
import krpc_geometry as geo
import krpc_judge_replica as judge


def perturb(rng, spec, max_offset, max_angle_deg):
    offset = [rng.uniform(-1, 1) for _ in range(3)]
    offset = [c * rng.uniform(0, max_offset) / max(1e-9, geo.norm(offset)) for c in offset]
    position = [spec.position[i] + offset[i] for i in range(3)]
    axis = [rng.gauss(0, 1) for _ in range(3)]
    delta = geo.quat_from_axis_angle(axis, math.radians(rng.uniform(0, max_angle_deg)))
    return position, geo.quat_multiply(delta, spec.target_quaternion)


class ParsingTest(unittest.TestCase):

    def test_minimal_payload_uses_defaults(self):
        message = json.dumps({'checkpoints': [{'checkpoint_id': 3, 'position': [1, 2, 3]}], 'goalpoint_id': 3})
        specs, goal_id = judge.parse_checkpoints(message)
        self.assertEqual(goal_id, 3)
        self.assertEqual(specs[0].score_bands, [(0.2, 3), (0.4, 2), (0.6, 1)])
        self.assertEqual(specs[0].centering_bands, [])

    def test_unknown_goal_and_bad_items(self):
        message = json.dumps({'checkpoints': [{'id': 1, 'position': [0, 0, 0]}, {'id': 2}, 'junk'],
                              'goalpoint_id': 99})
        specs, goal_id = judge.parse_checkpoints(message)
        self.assertEqual([s.checkpoint_id for s in specs], [1])
        self.assertIsNone(goal_id)

    def test_invalid_payload_raises(self):
        for message in ('[]', '{}', '{"checkpoints": []}'):
            with self.assertRaises(ValueError):
                judge.parse_checkpoints(message)

    def test_classify_and_best_threshold(self):
        bands = [(0.1, 3), (0.2, 2), (0.3, 1)]
        self.assertEqual([judge.classify(v, bands) for v in (0.0, 0.1, 0.15, 0.3, 0.31)], [3, 3, 2, 1, 0])
        self.assertEqual(judge.best_threshold(bands), 0.1)
        self.assertIsNone(judge.best_threshold([]))


@unittest.skipUnless(support.SIMULATOR_AVAILABLE, 'simulator sources (rootfs/) not found')
class ReplicaMatchesJudgeTest(unittest.TestCase):

    def test_generated_payloads_parse_with_goal(self):
        for level, count in (('easy', 6), ('normal', 8), ('hard', 10), ('veryhard', 12)):
            cheat = support.generate_cheat_sheet(level, seed=7)
            specs, goal_id = judge.parse_checkpoints(support.api_payload_json(cheat))
            self.assertEqual(len(specs), count)
            self.assertEqual(goal_id, 12)

    def test_ideal_pose_is_level3_for_judge_and_replica(self):
        for seed in range(30):
            cheat = support.generate_cheat_sheet('veryhard', seed)
            referee = support.make_referee(cheat)
            specs, _ = judge.parse_checkpoints(support.api_payload_json(cheat))
            for spec in specs:
                ours = judge.evaluate(spec, spec.position, spec.target_quaternion)
                success, _, detail = support.referee_evaluate(referee, spec.checkpoint_id,
                                                              spec.position, spec.target_quaternion)
                self.assertTrue(success)
                self.assertEqual(detail['selected_band'], 3)
                self.assertEqual(ours.level, 3)
                self.assertTrue(judge.within_margin(spec, ours, 0.7, 0.6))

    def test_perturbed_poses_match_judge_exactly(self):
        rng = random.Random(11)
        for seed in range(10):
            cheat = support.generate_cheat_sheet('veryhard', seed)
            referee = support.make_referee(cheat)
            specs, _ = judge.parse_checkpoints(support.api_payload_json(cheat))
            for spec in specs:
                for _ in range(40):
                    position, quaternion = perturb(rng, spec, 0.4, 40.0)
                    ours = judge.evaluate(spec, position, quaternion)
                    _, _, detail = support.referee_evaluate(referee, spec.checkpoint_id, position, quaternion)
                    self.assertAlmostEqual(ours.distance, detail['distance'], places=9)
                    self.assertAlmostEqual(ours.centering_deg, detail['centering_angle_deg'], places=6)
                    self.assertAlmostEqual(ours.roll_deg, detail['roll_angle_deg'], places=6)
                    self.assertEqual(ours.level, detail['selected_band'])

    def test_margin_implies_level3_with_tolerance(self):
        """Anything accepted by within_margin stays Lv3 even if the judge's pose sample
        differs by up to the remaining margin."""
        rng = random.Random(5)
        cheat = support.generate_cheat_sheet('veryhard', 1)
        specs, _ = judge.parse_checkpoints(support.api_payload_json(cheat))
        for spec in specs:
            for _ in range(200):
                position, quaternion = perturb(rng, spec, 0.1, 10.0)
                evaluation = judge.evaluate(spec, position, quaternion)
                if judge.within_margin(spec, evaluation, 0.7, 0.6):
                    self.assertEqual(evaluation.level, 3)


if __name__ == '__main__':
    unittest.main()
