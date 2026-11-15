# -*- coding:utf-8 -*-
"""
krpc_config.py - ALL the numbers you are allowed to tune, in one place.
=====================================================================

Why a separate file?
    The mission logic (user_template.py) should never contain "magic numbers".
    When you want the robot to be faster or more careful, you only change
    values here, run the simulator again, and compare the result.

How to tune safely (golden rule):
    Change ONE value at a time, run the simulator, and check
    share/logs/latest/result.json. If any checkpoint drops below level 3,
    undo the change.

Units used in this file:
    seconds [s], meters [m], degrees [deg], radians per second [rad/s].
"""

# Printed in the log at start-up, so you always know which version produced a result.
VERSION = 'vnm-ai-0.1.0'


# ===========================================================================
# 1. TIMEOUTS - how long we are willing to wait for other ROS programs
# ===========================================================================
# Rule from the Programming Manual: never wait forever. If something does not
# answer in time we log an error and stop cleanly instead of freezing.

SERVER_WAIT_TIMEOUT = 10.0       # [s] wait for an action server / service, per attempt
SERVER_WAIT_RETRIES = 3          # how many attempts before giving up
GET_CHECKPOINTS_RETRIES = 3      # attempts to download the checkpoint list
NAV_WAIT_TIMEOUT = 10.0          # [s] wait for the first position message
NAV_STALE_SEC = 0.3              # [s] a position older than this is "too old" to trust
CHECKPOINT_RESULT_TIMEOUT = 10.0 # [s] wait for the answer of /event/checkpoint_passed
MISSION_EVENT_TIMEOUT = 10.0     # [s] wait for mission_start / mission_finished services
CANCEL_WAIT_TIMEOUT = 2.0        # [s] wait for a cancelled movement to really stop
POLL_PERIOD = 0.05               # [s] how often loops check the robot state (20 times/s)


# ===========================================================================
# 2. MOVEMENT - how we ask the robot to fly (/ctl/command)
# ===========================================================================

TARGET_FRAME_ID = 'iss_body'     # coordinate frame for absolute targets (Manual 6.3.2)

# A movement that takes too long is cancelled. The allowed time for one flight
# ("leg") is:  estimated_time * MOVE_TIMEOUT_FACTOR + MOVE_TIMEOUT_MARGIN,
# but never less than MOVE_TIMEOUT_MIN.
MOVE_TIMEOUT_FACTOR = 3.0
MOVE_TIMEOUT_MARGIN = 20.0       # [s]
MOVE_TIMEOUT_MIN = 30.0          # [s]

MOVE_RETRIES = 2                 # extra tries per checkpoint if the pose is not good enough


# ===========================================================================
# 3. ROUTE PLANNING - guessing how long each flight takes
# ===========================================================================
# The planner picks the visiting order with the smallest total time.
# It estimates one flight as:
#
#     time = max(distance / V_EST, rotation_angle / W_EST) + LEG_OVERHEAD
#
# "max" because the robot moves and turns at the same time.
# Better guesses -> better order. Measure real values from the log lines that
# start with "leg done" (they print move time, distance and rotation).

V_EST = 0.10                     # [m/s]   average flying speed
W_EST = 0.30                     # [rad/s] average turning speed
LEG_OVERHEAD = 3.0               # [s]     speeding up, slowing down, checking, notifying


# ===========================================================================
# 4. CHECKPOINT VERIFICATION - when is the robot "good enough" to report?
# ===========================================================================
# The judge checks our pose ONCE per checkpoint and never again, so we only
# report when our own copy of the judge (krpc_judge_replica.py) says level 3.
#
# Safety margins: we demand to be better than the level-3 limit, because the
# judge reads the position a little later than we do (it samples at ~5 Hz).
#   Example: level 3 needs distance <= 0.10 m. With POS_MARGIN_RATIO = 0.7
#   we only report when distance <= 0.07 m.

POS_MARGIN_RATIO = 0.7           # distance must be <= 70% of the level-3 radius
ANG_MARGIN_RATIO = 0.6           # centering / roll must be <= 60% of the level-3 angle
V_STILL = 0.01                   # [m/s]   slower than this counts as "not moving"
W_STILL = 0.02                   # [rad/s] slower than this counts as "not rotating"
HOLD_SEC = 0.6                   # [s] the good pose must last this long before reporting
HOLD_MAX_DRIFT = 0.01            # [m] the robot may drift at most this much while holding
SETTLE_TIMEOUT = 8.0             # [s] give up waiting for a perfect pose after this
MIN_NOTIFY_LEVEL = 1             # if level 3 is impossible, still report when level >= this
FALLBACK_STILL_TIMEOUT = 3.0     # [s] wait for the robot to stop before a lower-level report


# ===========================================================================
# 5. MISSION TIME
# ===========================================================================
# The whole simulation is stopped after 10 minutes. If we are running late we
# skip the remaining normal checkpoints and fly straight to the goal, because
# reaching the goal is worth more than one checkpoint.

MISSION_TIME_BUDGET = 480.0      # [s] counted from mission_start


# ===========================================================================
# 6. EXPERIMENTAL SWITCHES - keep False until the basic version is stable
# ===========================================================================

# True  = report a checkpoint as soon as the pose is perfect, even while the
#         movement command is still "finishing". Can save time per checkpoint.
EARLY_NOTIFY = False

# True  = look at the camera image with OpenCV ArUco and log how far the marker
#         is from the image center. Only for debugging: the simulator judge does
#         not use the image at all.
ENABLE_VISION = False


# ===========================================================================
# 7. DEFAULTS copied from the simulator (only used if the API leaves them out)
# ===========================================================================

DEFAULT_SCORE_BANDS = [(0.20, 3), (0.40, 2), (0.60, 1)]   # (max distance [m], level)
DEFAULT_GOAL_ID = 12                                       # the goal is always marker 12
