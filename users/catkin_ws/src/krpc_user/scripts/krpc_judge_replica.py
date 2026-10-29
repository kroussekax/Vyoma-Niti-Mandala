# -*- coding:utf-8 -*-
"""
krpc_judge_replica.py - our own copy of the simulator's scoring rules.
======================================================================

How the real judge scores a checkpoint (rootfs/krpc/judge_module/judge.py):

    1. We send /event/checkpoint_passed with a checkpoint id.
    2. The judge takes ONE snapshot of the robot pose (/sensor_fusion/navigation).
    3. It checks three things, and each one gets a level 3, 2, 1 or 0 (= fail):

         a) DISTANCE   robot center  <->  checkpoint "position"
                       e.g. <=0.10 m -> Lv3, <=0.20 m -> Lv2, <=0.30 m -> Lv1
         b) CENTERING  angle between camera direction and the direction to the marker
                       e.g. <=10 deg -> Lv3, <=20 -> Lv2, <=30 -> Lv1
         c) ROLL       twist around the line of sight compared with the target attitude
                       e.g. <=10 deg -> Lv3, <=20 -> Lv2, <=30 -> Lv1

    4. Final level = the LOWEST of the three. If any of them is 0, the checkpoint fails.
    5. A checkpoint is judged only ONCE. Sending it again is ignored.

Because of rule 5, a report at the wrong moment loses that checkpoint forever.
So before reporting, we compute exactly the same numbers ourselves with this
file. The unit tests (test/test_judge_replica.py) prove our numbers equal the
judge's numbers.
"""

import json

import krpc_config as cfg
import krpc_geometry as geo


# ---------------------------------------------------------------------------
# "Bands" = the level table of one condition, e.g. [(0.1, 3), (0.2, 2), (0.3, 1)]
# ---------------------------------------------------------------------------

def _bands(band_objects, key_names):
    """Turn the JSON list [{"radius": 0.1, "level": 3}, ...] into [(0.1, 3), ...].

    `key_names` lists the possible names of the threshold field ("radius" for
    distance, "angle" for attitude). The result is sorted from strict to loose,
    exactly like the judge sorts it."""
    bands = []
    if not isinstance(band_objects, list):
        return bands
    for band in band_objects:
        if not isinstance(band, dict):
            continue
        threshold = None
        for key in key_names:
            if key in band:
                threshold = band[key]
                break
        level = band.get('level', band.get('score', band.get('point', band.get('points'))))
        try:
            bands.append((float(threshold), int(level)))
        except (TypeError, ValueError):
            continue                               # skip a broken entry, keep the rest
    bands.sort(key=lambda b: b[0])
    return bands


def classify(value, bands):
    """Level for a measured value. Example with [(0.1, 3), (0.2, 2), (0.3, 1)]:
    0.05 -> 3, 0.15 -> 2, 0.25 -> 1, 0.5 -> 0 (fail)."""
    for threshold, level in bands:
        if value <= threshold:
            return level
    return 0


def best_threshold(bands):
    """The limit you must be under to get the HIGHEST level (None if no bands)."""
    if not bands:
        return None
    return max(bands, key=lambda b: (b[1], -b[0]))[0]


def _vec3(value):
    """[x, y, z] as floats, or None if the JSON value is not a valid 3D point."""
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return [float(value[0]), float(value[1]), float(value[2])]
    return None


def _quat(value):
    """[x, y, z, w] as a normalized quaternion, or None if invalid."""
    if isinstance(value, dict):
        value = [value.get('x', 0.0), value.get('y', 0.0), value.get('z', 0.0), value.get('w', 1.0)]
    if isinstance(value, (list, tuple)) and len(value) >= 4:
        return geo.quat_normalize([float(v) for v in value[:4]])
    return None


# ---------------------------------------------------------------------------
# One checkpoint
# ---------------------------------------------------------------------------

class CheckpointSpec(object):
    """Everything we know about one checkpoint (from /mission/get_checkpoints).

    checkpoint_id      int, same number as the AR marker id
    position           [x, y, z] where the robot center should be
    marker_position    [x, y, z] of the AR marker the camera must look at
    target_quaternion  attitude chosen by the judge (already looks at the marker
                       and contains the random target roll)
    score_bands        distance level table
    centering_bands    centering level table (empty = not judged)
    roll_bands         roll level table (empty = not judged)
    """

    def __init__(self, checkpoint_id, position, marker_position, target_quaternion,
                 score_bands, centering_bands, roll_bands):
        self.checkpoint_id = checkpoint_id
        self.position = position
        self.marker_position = marker_position
        self.target_quaternion = target_quaternion
        self.score_bands = score_bands
        self.centering_bands = centering_bands
        self.roll_bands = roll_bands

    @classmethod
    def from_payload_item(cls, item):
        """Build from one element of the 'checkpoints' list. Returns None if unusable."""
        if not isinstance(item, dict):
            return None
        checkpoint_id = None
        for key in ('checkpoint_id', 'checkpointid', 'id'):    # the API has used all three names
            if item.get(key) is not None:
                checkpoint_id = int(item[key])
                break
        position = _vec3(item.get('position'))
        if checkpoint_id is None or position is None:
            return None

        marker_pose = item.get('marker_pose') if isinstance(item.get('marker_pose'), dict) else {}
        # Same fallback as the judge: no marker given -> use the checkpoint position.
        marker_position = _vec3(marker_pose.get('position')) or list(position)

        score_bands = _bands(item.get('score_bands', item.get('bands')), ('radius', 'margin', 'distance'))
        if not score_bands:
            score_bands = list(cfg.DEFAULT_SCORE_BANDS)

        attitude = item.get('attitude') if isinstance(item.get('attitude'), dict) else {}
        attitude_keys = ('angle', 'margin', 'threshold')
        target_quaternion = _quat(attitude.get('quaternion'))
        centering_bands = _bands(attitude.get('centering_bands'), attitude_keys)
        roll_bands = _bands(attitude.get('roll_bands'), attitude_keys)
        if target_quaternion is None:
            # No attitude rule: the judge only checks distance. We still look at
            # the marker because it costs nothing.
            target_quaternion = geo.look_at_quaternion(geo.sub(marker_position, position))
            centering_bands, roll_bands = [], []

        return cls(checkpoint_id, position, marker_position, target_quaternion,
                   score_bands, centering_bands, roll_bands)


class Evaluation(object):
    """Result of checking one pose against one checkpoint.

    Every *_level is 3, 2, 1, 0, or None (None = that condition is not judged).
    `level` is the final level (lowest of the three, 0 = fail)."""

    def __init__(self, distance, distance_level, centering_deg, centering_level, roll_deg, roll_level):
        self.distance = distance
        self.distance_level = distance_level
        self.centering_deg = centering_deg
        self.centering_level = centering_level
        self.roll_deg = roll_deg
        self.roll_level = roll_level
        levels = [lv for lv in (distance_level, centering_level, roll_level) if lv is not None]
        self.level = 0 if any(lv == 0 for lv in levels) else min(levels)

    def __str__(self):
        """Readable one-line summary for the log."""
        def fmt(value, unit):
            return '-' if value is None else '%.3f%s' % (value, unit)
        return 'level=%d dist=%s(L%s) center=%s(L%s) roll=%s(L%s)' % (
            self.level,
            fmt(self.distance, 'm'), self.distance_level,
            fmt(self.centering_deg, 'deg'), self.centering_level,
            fmt(self.roll_deg, 'deg'), self.roll_level)


def evaluate(spec, position, quaternion):
    """Score a robot pose exactly like the judge would (regular mode)."""
    dist = geo.distance(position, spec.position)
    centering_deg = roll_deg = centering_level = roll_level = None
    if spec.centering_bands or spec.roll_bands:
        centering_deg, roll_deg = geo.centering_and_roll_deg(
            position, quaternion, spec.marker_position, spec.target_quaternion)
        if spec.centering_bands:
            centering_level = classify(centering_deg, spec.centering_bands)
        if spec.roll_bands:
            roll_level = classify(roll_deg, spec.roll_bands)
    return Evaluation(dist, classify(dist, spec.score_bands),
                      centering_deg, centering_level, roll_deg, roll_level)


def within_margin(spec, evaluation, pos_ratio, ang_ratio):
    """Stricter than level 3: every value must stay below ratio * (level-3 limit).
    This leaves room for the small delay between our check and the judge's check."""
    radius = best_threshold(spec.score_bands)
    if radius is None or evaluation.distance > pos_ratio * radius:
        return False
    for value, bands in ((evaluation.centering_deg, spec.centering_bands),
                         (evaluation.roll_deg, spec.roll_bands)):
        threshold = best_threshold(bands)
        if threshold is not None and (value is None or value > ang_ratio * threshold):
            return False
    return True


def parse_checkpoints(message):
    """Read the JSON text from /mission/get_checkpoints.

    Returns (list of CheckpointSpec sorted by id, goal checkpoint id or None).
    Raises ValueError when the text is not usable at all."""
    payload = json.loads(message)
    if not isinstance(payload, dict):
        raise ValueError('checkpoint payload must be a JSON object')
    items = payload.get('checkpoints', payload.get('Checkpoints'))
    if not isinstance(items, list):
        raise ValueError('checkpoint payload has no checkpoints array')

    specs = {}
    for item in items:
        spec = CheckpointSpec.from_payload_item(item)
        if spec is not None:
            specs[spec.checkpoint_id] = spec
    if not specs:
        raise ValueError('no usable checkpoints in payload')

    goal_id = payload.get('goalpoint_id')
    goal_id = int(goal_id) if goal_id is not None else cfg.DEFAULT_GOAL_ID
    if goal_id not in specs:
        goal_id = None                           # unknown goal: we just visit everything
    return [specs[k] for k in sorted(specs)], goal_id
