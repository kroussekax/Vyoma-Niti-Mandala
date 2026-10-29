# -*- coding:utf-8 -*-
"""Shared helpers for the offline tests (no ROS needed).

The tests compare our modules with the simulator's REAL judge.py and
generate_objects.py from this repository. ROS imports are replaced with fake
"mock" modules, so everything runs on a normal computer.

Run all tests from the krpc_user folder:
    python3 -m unittest discover -s test -v
"""

import contextlib
import importlib.util
import io
import json
import os
import random
import sys
import warnings
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(os.path.dirname(HERE), 'scripts')
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)


def _find_repo_root():
    path = HERE
    while path != os.path.dirname(path):
        if os.path.isdir(os.path.join(path, 'rootfs', 'krpc')):
            return path
        path = os.path.dirname(path)
    return None


REPO_ROOT = _find_repo_root()
JUDGE_PATH = REPO_ROOT and os.path.join(REPO_ROOT, 'rootfs', 'krpc', 'judge_module', 'judge.py')
GENERATOR_DIR = REPO_ROOT and os.path.join(REPO_ROOT, 'rootfs', 'krpc', 'random_env_generator')
LOADER_DIR = REPO_ROOT and os.path.join(REPO_ROOT, 'users', 'catkin_ws', 'src', 'krpc_api_server', 'scripts')
SIMULATOR_AVAILABLE = bool(REPO_ROOT)

_ROS_STUBS = ('rospy', 'tf', 'requests', 'geometry_msgs', 'geometry_msgs.msg',
              'std_msgs', 'std_msgs.msg', 'ib2_msgs', 'ib2_msgs.msg')
_cache = {}


def _load_module(name, path, extra_path=None, stubs=()):
    if name in _cache:
        return _cache[name]
    saved = dict((s, sys.modules.get(s)) for s in stubs)
    for s in stubs:
        sys.modules[s] = mock.MagicMock()
    if extra_path:
        sys.path.insert(0, extra_path)
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')          # old simulator code, not our problem
            spec.loader.exec_module(module)
    finally:
        if extra_path:
            sys.path.remove(extra_path)
        for s, original in saved.items():
            if original is None:
                sys.modules.pop(s, None)
            else:
                sys.modules[s] = original
    _cache[name] = module
    return module


def judge_module():
    return _load_module('krpc_sim_judge', JUDGE_PATH, stubs=_ROS_STUBS)


def generator_module():
    return _load_module('krpc_sim_generator', os.path.join(GENERATOR_DIR, 'generate_objects.py'),
                        extra_path=GENERATOR_DIR)


def loader_module():
    return _load_module('krpc_sim_loader', os.path.join(LOADER_DIR, 'cheat_sheet_loader.py'))


def generate_cheat_sheet(level='veryhard', seed=0):
    """Same steps as generate_objects.main(), deterministic via seed."""
    gen = generator_module()
    random.seed(seed)
    with open(os.path.join(GENERATOR_DIR, 'files', 'input', 'area_config.json')) as f:
        area_config = json.load(f)
    params = {
        'distance': gen.DEFAULT_CHECKPOINT_DISTANCE,
        'yaw_min_deg': gen.DEFAULT_CHECKPOINT_YAW_MIN_DEG,
        'yaw_max_deg': gen.DEFAULT_CHECKPOINT_YAW_MAX_DEG,
        'pitch_min_deg': gen.DEFAULT_CHECKPOINT_PITCH_MIN_DEG,
        'pitch_max_deg': gen.DEFAULT_CHECKPOINT_PITCH_MAX_DEG,
        'roll_min_deg': gen.DEFAULT_CHECKPOINT_ROLL_MIN_DEG,
        'roll_max_deg': gen.DEFAULT_CHECKPOINT_ROLL_MAX_DEG,
    }
    markers = gen.select_markers_by_difficulty(area_config['UrdfMarkers'], level, gen.GOAL_MARKER_ID)
    with contextlib.redirect_stdout(io.StringIO()):  # the generator prints a lot
        checkpoints, goal = gen.build_checkpoints_from_urdf_markers(markers, area_config, params, gen.GOAL_MARKER_ID)
    return {'Level': level, 'Checkpoints': checkpoints, 'Goal': goal}


def api_payload_json(cheat_sheet):
    """What /mission/get_checkpoints returns for this cheat sheet (via the real loader)."""
    loader = loader_module().CheatSheetLoader(cheat_sheet, '<memory>')
    return json.dumps(loader.checkpoint_payload())


class _Vec(object):
    def __init__(self, **kw):
        self.__dict__.update(kw)


def make_referee(cheat_sheet):
    """A judge.Referee wired to cheat_sheet without touching ROS or files."""
    judge = judge_module()
    referee = object.__new__(judge.Referee)
    referee.sim_mode = 'regular'
    referee.cheat_data = cheat_sheet
    referee._checkpoint_rules, referee._checkpoint_defaults = referee.load_checkpoint_rules()
    referee.ros_time_string = lambda *args: ''
    return referee


def referee_evaluate(referee, checkpoint_id, position, quaternion):
    referee.ib2_pose = _Vec(
        position=_Vec(x=position[0], y=position[1], z=position[2]),
        orientation=_Vec(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]))
    return referee.evaluate_checkpoint_navigation(checkpoint_id)
