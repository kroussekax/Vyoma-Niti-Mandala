#!/usr/bin/python3
# -*- coding:utf-8 -*-
"""
user_template.py - the main program (ROS node) of our Kibo-RPC 7 team.
======================================================================

The simulator starts this file (see launch/krpc_user.launch). The file name
must stay "user_template.py".

What the mission looks like
---------------------------
    PREPARE (before the timer starts, so it costs no score time)
      1. wait until the movement and checkpoint servers are online
      2. download the checkpoint list   (/mission/get_checkpoints)
      3. read where the robot is now    (/sensor_fusion/navigation)
      4. compute the fastest visiting order (krpc_route_planner.py)

    MISSION (the timer is running)
      5. /event/mission_start
      6. for every checkpoint in the planned order:
           a. fly there                         (/ctl/command)
           b. wait until our judge copy says Lv3 and the robot is still
           c. report it                         (/event/checkpoint_passed)
      7. the goal (marker 12) is the last checkpoint
      8. /event/mission_finished

Rules this file must ALWAYS keep (or the run is invalid)
-------------------------------------------------------
    * subscribe to /ib2_user/start         (lines marked DO NOT REMOVE)
    * provide the /ib2_user/stop service   (lines marked DO NOT REMOVE)
    * set the /ib2_user/ready parameter to True at the end of __init__

Where to read next
------------------
    krpc_config.py         all tunable numbers
    krpc_judge_replica.py  how a checkpoint is scored
    krpc_route_planner.py  how the order is chosen
    krpc_geometry.py       the 3D math
    README.md              the full explanation for new team members

A note on threads
-----------------
ROS calls our "callback" functions (for example __callback_nav) from its own
background threads, while the mission runs in the start callback. Data shared
between them is protected with a Lock so it is never half-written.
"""
import os
import sys
import threading
import time
import traceback
from collections import namedtuple

# ROS runs this file through a small wrapper script in devel/lib/, so Python
# would not find our krpc_*.py files by itself. Add this folder to the search path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
import actionlib
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from geometry_msgs.msg import PoseStamped
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType, CtlStatus, Navigation

import krpc_config as cfg
import krpc_geometry as geo
import krpc_judge_replica as judge
import krpc_route_planner as planner

# One reading of the robot state. A namedtuple is a small read-only record:
# nav.position, nav.quaternion, ...
#   stamp          ROS time when we received it [s]
#   position       [x, y, z] [m]
#   quaternion     [x, y, z, w]
#   linear_speed   [m/s]
#   angular_speed  [rad/s]
NavSnapshot = namedtuple('NavSnapshot', 'stamp position quaternion linear_speed angular_speed')


class MissionAborted(Exception):
    """Raised when /ib2_user/stop is received or ROS shuts down.
    It jumps straight out of any loop, so the mission stops quickly and cleanly."""


class HoldTracker(object):
    """Answers the question "has the pose been good for HOLD_SEC seconds in a row?"

    Call update() with every new reading. Any bad reading, or drifting more than
    HOLD_MAX_DRIFT meters, restarts the stopwatch."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.__since = None       # time when the good streak started
        self.__anchor = None      # position at the start of the streak

    def update(self, good, now, position):
        if not good:
            self.reset()
            return False
        if self.__since is None or geo.distance(self.__anchor, position) > cfg.HOLD_MAX_DRIFT:
            self.__since = now
            self.__anchor = position
        return now - self.__since >= cfg.HOLD_SEC


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')

        # Action clients: for long tasks that report back when finished
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # Service clients: for quick question -> answer calls
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # State written by callbacks and read by the mission (see "A note on threads")
        self.__nav_lock = threading.Lock()
        self.__nav = None                  # latest NavSnapshot
        self.__nav_count = 0               # number of nav messages received (to measure the rate)
        self.__ctl_status_type = None      # latest controller state (KEEP_POSE, MOVE_TO_..., ...)
        self.__stop_requested = threading.Event()
        self.__mission_start_time = None

        self.__vision = None
        if cfg.ENABLE_VISION:
            import krpc_vision
            self.__vision = krpc_vision.MarkerObserver()

        # Subscribers
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE
        self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav, queue_size=1)
        self.__subscriber_ctl_status = rospy.Subscriber('/ctl/status', CtlStatus, self.__callback_ctl_status, queue_size=1)

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing) # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False               # True while a movement goal is running
        self.__checkpoint_action_active = False    # True while a checkpoint report is running
        rospy.set_param('/ib2_user/ready', True) # DO NOT REMOVE THIS LINE

    # ==================================================================
    # Small helpers
    # ==================================================================
    def __check_stop(self):
        """Call often inside loops: leaves the mission at once if we must stop."""
        if self.__stop_requested.is_set() or rospy.is_shutdown():
            raise MissionAborted('stop requested' if self.__stop_requested.is_set() else 'ros shutdown')

    def __sleep(self, seconds):
        """rospy.sleep that also reacts to stop requests."""
        self.__check_stop()
        try:
            rospy.sleep(seconds)
        except rospy.ROSInterruptException:
            raise MissionAborted('ros shutdown')

    def __nav_snapshot(self):
        """Latest robot state, or None if we have none or it is too old to trust."""
        with self.__nav_lock:
            nav = self.__nav
        if nav is None or rospy.get_time() - nav.stamp > cfg.NAV_STALE_SEC:
            return None
        return nav

    def __is_still(self, nav):
        """True when the robot is (almost) not moving and not rotating."""
        return nav.linear_speed <= cfg.V_STILL and nav.angular_speed <= cfg.W_STILL

    def __elapsed(self):
        """Seconds since mission_start (0 before the mission started)."""
        if self.__mission_start_time is None:
            return 0.0
        return rospy.get_time() - self.__mission_start_time

    @staticmethod
    def __fmt(values):
        """Format a list of numbers for the log: [1.2345, 0.0000, ...]."""
        return '[' + ', '.join('%.4f' % v for v in values) + ']'

    # ==================================================================
    # Talking to the game API (always with a time limit)
    # ==================================================================
    def __wait_server(self, client, name):
        """Wait (with retries) until an action server is online."""
        for attempt in range(1, cfg.SERVER_WAIT_RETRIES + 1):
            self.__check_stop()
            if client.wait_for_server(rospy.Duration(cfg.SERVER_WAIT_TIMEOUT)):
                return True
            rospy.logwarn('action server %s not available (attempt %d/%d)', name, attempt, cfg.SERVER_WAIT_RETRIES)
        rospy.logerr('action server %s unavailable, giving up', name)
        return False

    def __call_service(self, proxy, name, retries=1):
        """Call a service that answers (success, message). Returns the answer, or None on failure."""
        for attempt in range(1, retries + 1):
            self.__check_stop()
            try:
                rospy.wait_for_service(name, timeout=cfg.MISSION_EVENT_TIMEOUT)
                res = proxy()
                rospy.loginfo('%s result: success=%s message=%.200s', name, res.success, res.message)
                if res.success:
                    return res
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn('%s failed (attempt %d/%d): %s', name, attempt, retries, exc)
        return None

    def __notify_mission_start(self):
        return self.__call_service(self.__mission_start, '/event/mission_start') is not None

    def __notify_mission_finished(self):
        return self.__call_service(self.__mission_finished, '/event/mission_finished') is not None

    def __notify_checkpoint_passed(self, checkpoint_id):
        """Report a checkpoint. The judge scores the pose at this moment, ONCE."""
        rospy.loginfo('notify checkpoint_id=%d passed (t=%.1fs)', checkpoint_id, self.__elapsed())
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self.__checkpoint_action_active = True
        try:
            self.__checkpoint_client.send_goal(goal)
            deadline = rospy.get_time() + cfg.CHECKPOINT_RESULT_TIMEOUT
            while rospy.get_time() < deadline:
                self.__check_stop()
                if self.__checkpoint_client.wait_for_result(rospy.Duration(cfg.POLL_PERIOD)):
                    result = self.__checkpoint_client.get_result()
                    rospy.loginfo('checkpoint_passed result: id=%d success=%s message=%s',
                                  checkpoint_id, result.success if result else None,
                                  result.message if result else None)
                    return bool(result and result.success)
            rospy.logwarn('checkpoint_passed id=%d timed out', checkpoint_id)
            self.__checkpoint_client.cancel_goal()
            return False
        except MissionAborted:
            self.__checkpoint_client.cancel_goal()
            raise
        finally:
            self.__checkpoint_action_active = False

    # ==================================================================
    # PREPARE: runs before mission_start, so it does not cost mission time
    # ==================================================================
    def __load_checkpoints(self):
        """Download and parse the checkpoint list. Returns (specs, goal_id) or (None, None)."""
        res = self.__call_service(self.__checkpoints, '/mission/get_checkpoints', cfg.GET_CHECKPOINTS_RETRIES)
        if res is None:
            return None, None
        try:
            specs, goal_id = judge.parse_checkpoints(res.message)
        except (ValueError, TypeError, KeyError) as exc:
            rospy.logerr('invalid checkpoint payload: %s', exc)
            return None, None
        for spec in specs:
            rospy.loginfo('checkpoint %d: pos=%s marker=%s q=%s bands=%s/%s/%s',
                          spec.checkpoint_id, self.__fmt(spec.position), self.__fmt(spec.marker_position),
                          self.__fmt(spec.target_quaternion), spec.score_bands, spec.centering_bands, spec.roll_bands)
        rospy.loginfo('goal checkpoint id=%s, %d checkpoints', goal_id, len(specs))
        return specs, goal_id

    def __wait_nav(self):
        """Wait for the first fresh position message."""
        deadline = rospy.get_time() + cfg.NAV_WAIT_TIMEOUT
        while rospy.get_time() < deadline:
            nav = self.__nav_snapshot()
            if nav is not None:
                return nav
            self.__sleep(cfg.POLL_PERIOD)
        rospy.logerr('no fresh /sensor_fusion/navigation within %.1fs', cfg.NAV_WAIT_TIMEOUT)
        return None

    def __prepare(self):
        """Steps 1-4 of the mission. Returns (specs_by_id, order, goal_id) or None."""
        rospy.loginfo('controller version %s', cfg.VERSION)
        if not self.__wait_server(self.__command, '/ctl/command'):
            return None
        if not self.__wait_server(self.__checkpoint_client, '/event/checkpoint_passed'):
            return None
        specs, goal_id = self.__load_checkpoints()
        if not specs:
            return None

        # Measure how often nav messages arrive (useful when tuning NAV_STALE_SEC).
        count_before = self.__nav_count
        stamp_before = rospy.get_time()
        nav = self.__wait_nav()
        if nav is None:
            return None
        self.__sleep(1.0)
        rate = (self.__nav_count - count_before) / max(1e-3, rospy.get_time() - stamp_before)
        nav = self.__nav_snapshot() or nav
        rospy.loginfo('start pose=%s q=%s (nav %.1f Hz)', self.__fmt(nav.position), self.__fmt(nav.quaternion), rate)

        # Every normal checkpoint can be visited in any order; the goal must be last.
        specs_by_id = dict((s.checkpoint_id, s) for s in specs)
        nodes = [(s.checkpoint_id, s.position, s.target_quaternion) for s in specs if s.checkpoint_id != goal_id]
        goal_node = None
        if goal_id is not None:
            goal = specs_by_id[goal_id]
            goal_node = (goal_id, goal.position, goal.target_quaternion)
        plan_start = time.time()
        order, estimate = planner.plan_route((nav.position, nav.quaternion), nodes, goal_node)
        rospy.loginfo('route %s, estimated %.1fs (planned in %.3fs)', order, estimate, time.time() - plan_start)
        return specs_by_id, order, goal_id

    # ==================================================================
    # MOVE + VERIFY: one checkpoint
    # ==================================================================
    def __move_to(self, position, quaternion, timeout, early_spec=None):
        """Ask the robot to fly to an absolute pose and wait until it is done.

        Returns a pair (result, evaluation):
          result = CtlCommandResult.TERMINATE_SUCCESS / other code from the controller,
                   'early' (EARLY_NOTIFY found a perfect pose before the move ended),
                   or None (timeout, the move was cancelled)
          evaluation = the judge-replica result when result == 'early', else None
        """
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = cfg.TARGET_FRAME_ID
        target.pose.position.x, target.pose.position.y, target.pose.position.z = position
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = quaternion
        goal = CtlCommandGoal(target=target, type=CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET))

        tracker = HoldTracker()
        self.__action_active = True
        try:
            self.__command.send_goal(goal)
            deadline = rospy.get_time() + timeout
            while rospy.get_time() < deadline:
                self.__check_stop()
                if self.__command.wait_for_result(rospy.Duration(cfg.POLL_PERIOD)):
                    result = self.__command.get_result()
                    # NOTE: the result field is called "type" (not "status")
                    return (result.type if result is not None else None), None
                if early_spec is not None:
                    held, evaluation = self.__scorable_now(early_spec, tracker)
                    if held:
                        # Leave the goal running; the next send_goal() replaces it.
                        return 'early', evaluation
            rospy.logwarn('move timed out after %.1fs, cancelling', timeout)
            self.__cancel_command()
            return None, None
        except MissionAborted:
            self.__cancel_command()
            raise
        finally:
            self.__action_active = False

    def __cancel_command(self):
        """Cancel the current movement and wait (briefly) until it really ended."""
        self.__command.cancel_goal()
        self.__command.wait_for_result(rospy.Duration(cfg.CANCEL_WAIT_TIMEOUT))

    def __scorable_now(self, spec, tracker):
        """Check the latest pose with our judge copy.

        Returns (held, evaluation). held is True only when the pose is Lv3 WITH
        safety margin, the robot is still, and this has lasted HOLD_SEC seconds."""
        nav = self.__nav_snapshot()
        if nav is None:
            tracker.reset()
            return False, None
        evaluation = judge.evaluate(spec, nav.position, nav.quaternion)
        good = (evaluation.level == 3
                and judge.within_margin(spec, evaluation, cfg.POS_MARGIN_RATIO, cfg.ANG_MARGIN_RATIO)
                and self.__is_still(nav))
        return tracker.update(good, nav.stamp, nav.position), evaluation

    def __settle_and_verify(self, spec):
        """After a move: wait up to SETTLE_TIMEOUT for a scorable pose.
        Returns (ok, last evaluation)."""
        tracker = HoldTracker()
        deadline = rospy.get_time() + cfg.SETTLE_TIMEOUT
        evaluation = None
        while True:
            held, latest = self.__scorable_now(spec, tracker)
            evaluation = latest or evaluation
            if held:
                return True, evaluation
            if rospy.get_time() >= deadline:
                return False, evaluation
            self.__sleep(cfg.POLL_PERIOD)

    def __still_evaluation(self, spec):
        """Last resort: wait for the robot to stop moving and score that pose."""
        deadline = rospy.get_time() + cfg.FALLBACK_STILL_TIMEOUT
        while rospy.get_time() < deadline:
            nav = self.__nav_snapshot()
            if nav is not None and self.__is_still(nav):
                return judge.evaluate(spec, nav.position, nav.quaternion)
            self.__sleep(cfg.POLL_PERIOD)
        return None

    def __visit_checkpoint(self, spec):
        """Fly to one checkpoint and report it only when the score is known to be good.

        Returns the level we reported (3, 2, 1) or 0 if we did not report."""
        target_quaternion = spec.target_quaternion
        for attempt in range(1 + cfg.MOVE_RETRIES):
            self.__check_stop()
            start_nav = self.__nav_snapshot()
            if attempt > 0 and start_nav is not None:
                # Retry: aim the camera from where the robot really is, keeping the target roll.
                target_quaternion = geo.look_at_with_roll(start_nav.position, spec.marker_position, spec.target_quaternion)

            # Give the move a time limit based on how long we expect it to take.
            timeout = cfg.MOVE_TIMEOUT_MIN
            if start_nav is not None:
                estimate = planner.leg_time(start_nav.position, start_nav.quaternion, spec.position, target_quaternion)
                timeout = max(cfg.MOVE_TIMEOUT_MIN, estimate * cfg.MOVE_TIMEOUT_FACTOR + cfg.MOVE_TIMEOUT_MARGIN)

            leg_start = rospy.get_time()
            result, early_evaluation = self.__move_to(spec.position, target_quaternion, timeout,
                                                      early_spec=spec if cfg.EARLY_NOTIFY else None)
            move_time = rospy.get_time() - leg_start
            if start_nav is not None:
                # These numbers are what you use to tune V_EST / W_EST in krpc_config.py.
                rospy.loginfo('leg done id=%d attempt=%d result=%s move=%.2fs dist=%.3fm rot=%.1fdeg',
                              spec.checkpoint_id, attempt, result, move_time,
                              geo.distance(start_nav.position, spec.position),
                              geo.quat_angle_deg(start_nav.quaternion, target_quaternion))
            if result not in (CtlCommandResult.TERMINATE_SUCCESS, 'early'):
                rospy.logwarn('move to %d ended with result=%s, verifying anyway', spec.checkpoint_id, result)

            if result == 'early':
                ok, evaluation = True, early_evaluation
            else:
                ok, evaluation = self.__settle_and_verify(spec)
            rospy.loginfo('verify id=%d ok=%s settle=%.2fs %s', spec.checkpoint_id, ok,
                          rospy.get_time() - leg_start - move_time, evaluation)
            if ok:
                self.__log_vision(spec)
                self.__notify_checkpoint_passed(spec.checkpoint_id)
                return evaluation.level

        # Level 3 was not reached after all attempts. A lower level still scores
        # points, and that is better than zero.
        evaluation = self.__still_evaluation(spec)
        if evaluation is not None and evaluation.level >= cfg.MIN_NOTIFY_LEVEL:
            rospy.logwarn('checkpoint %d: notifying below target precision: %s', spec.checkpoint_id, evaluation)
            self.__notify_checkpoint_passed(spec.checkpoint_id)
            return evaluation.level
        rospy.logerr('checkpoint %d: not scorable, skipped (%s)', spec.checkpoint_id, evaluation)
        return 0

    def __log_vision(self, spec):
        if self.__vision is not None:
            rospy.loginfo('vision id=%d: %s', spec.checkpoint_id, self.__vision.measure(spec.checkpoint_id))

    # ==================================================================
    # ROUTE: all checkpoints
    # ==================================================================
    def __over_budget(self, spec, goal_spec):
        """True if visiting `spec` and then reaching the goal would exceed MISSION_TIME_BUDGET."""
        nav = self.__nav_snapshot()
        if nav is None or goal_spec is None:
            return False
        remaining = (planner.leg_time(nav.position, nav.quaternion, spec.position, spec.target_quaternion)
                     + planner.leg_time(spec.position, spec.target_quaternion, goal_spec.position, goal_spec.target_quaternion))
        return self.__elapsed() + remaining > cfg.MISSION_TIME_BUDGET

    def __execute_route(self, specs_by_id, order, goal_id):
        """Visit every checkpoint in the planned order. Returns a summary for the log."""
        summary = []
        goal_spec = specs_by_id.get(goal_id)
        for checkpoint_id in order:
            spec = specs_by_id[checkpoint_id]
            if checkpoint_id != goal_id and self.__over_budget(spec, goal_spec):
                rospy.logwarn('time budget: skipping checkpoint %d (t=%.1fs)', checkpoint_id, self.__elapsed())
                summary.append((checkpoint_id, 0, 'skipped(time)'))
                continue
            level = self.__visit_checkpoint(spec)
            summary.append((checkpoint_id, level, '%.1fs' % self.__elapsed()))
        return summary

    # ==================================================================
    # MISSION FLOW
    # ==================================================================
    def __run_user_flow(self, msg):
        summary = []
        try:
            plan = self.__prepare()
            if plan is None:
                rospy.logerr('preparation failed, mission not started')
                return
            specs_by_id, order, goal_id = plan

            if not self.__notify_mission_start():
                return
            self.__mission_start_time = rospy.get_time()

            # USER EDIT AREA: route execution (see README.md, "Architecture").
            # ***********************************************
            summary = self.__execute_route(specs_by_id, order, goal_id)
            # ***********************************************

            self.__check_stop()
            self.__notify_mission_finished()
        except MissionAborted as exc:
            rospy.logwarn('mission aborted: %s', exc)
            self.__command.cancel_goal()
        except Exception as exc:
            rospy.logerr('user flow failed: %s\n%s', exc, traceback.format_exc())
        finally:
            for checkpoint_id, level, note in summary:
                rospy.loginfo('summary checkpoint %d: level=%d %s', checkpoint_id, level, note)
            rospy.loginfo('mission time %.1fs', self.__elapsed())

    # ==================================================================
    # ROS callbacks
    # ==================================================================
    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    def __callback_nav(self, data):
        """Store the newest robot position, attitude and speed."""
        p = data.pose.pose.position
        o = data.pose.pose.orientation
        lin = data.twist.linear
        ang = data.twist.angular
        nav = NavSnapshot(rospy.get_time(), [p.x, p.y, p.z], [o.x, o.y, o.z, o.w],
                          geo.norm([lin.x, lin.y, lin.z]), geo.norm([ang.x, ang.y, ang.z]))
        with self.__nav_lock:
            self.__nav = nav
            self.__nav_count += 1

    def __callback_ctl_status(self, data):
        """Watch the controller state. A collision warning is worth seeing in the log."""
        status_type = data.type.type
        if status_type != self.__ctl_status_type:
            self.__ctl_status_type = status_type
            if status_type == CtlStatusType.KEEPING_POSE_BY_COLLISION:
                rospy.logwarn('ctl status: KEEPING_POSE_BY_COLLISION')
            else:
                rospy.logdebug('ctl status: %d', status_type)

    def __stop_processing(self, request):
        """/ib2_user/stop: stop everything, then answer SUCCESS (mandatory interface)."""
        try:
            self.__stop_requested.set()
            if self.__action_active:
                self.__command.cancel_goal()
            if self.__checkpoint_action_active:
                self.__checkpoint_client.cancel_goal()
            # The mission thread clears the flags once its cancel has completed.
            deadline = time.time() + cfg.CANCEL_WAIT_TIMEOUT
            while (self.__action_active or self.__checkpoint_action_active) and time.time() < deadline:
                time.sleep(0.05)
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()
