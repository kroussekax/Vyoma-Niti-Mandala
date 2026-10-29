# -*- coding:utf-8 -*-
"""
krpc_vision.py - OPTIONAL camera check with ArUco markers (debug tool).
=======================================================================

Turned on with ENABLE_VISION = True in krpc_config.py. It is off by default.

What it does
    Finds the AR marker in the latest camera image and logs how far the marker
    center is from the image center, in pixels and in degrees.

Why it is optional
    The simulator judge scores the robot POSE, not the picture. Vision therefore
    does not change the simulator score. It is useful to double-check centering
    by eye and as a starting point for the real Int-Ball2 robot.

Marker facts (Programming Manual 5.13)
    dictionary DICT_4X4_250, ids 1..12, side length 0.02 m, OpenCV 3.3.1 ArUco.
"""

import math
import threading

import rospy
from sensor_msgs.msg import Image, CameraInfo

ARUCO_DICT_ID = 'DICT_4X4_250'
MARKER_LENGTH = 0.02  # [m] (needed only if you add pose estimation later)


class MarkerObserver(object):

    def __init__(self):
        self.__lock = threading.Lock()       # camera callbacks run in other threads
        self.__image = None                  # latest sensor_msgs/Image
        self.__info = None                   # latest sensor_msgs/CameraInfo (K = camera matrix)
        self.__cv2 = None
        self.__np = None
        try:
            # Imported here, so the robot still flies if OpenCV is missing or broken.
            import cv2
            import numpy
            self.__cv2 = cv2
            self.__np = numpy
            self.__dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, ARUCO_DICT_ID))
            self.__parameters = cv2.aruco.DetectorParameters_create()
        except Exception as exc:
            rospy.logwarn('vision disabled: OpenCV/ArUco unavailable (%s)', exc)
            self.__cv2 = None
            return
        rospy.Subscriber('/camera_main/image_raw', Image, self.__callback_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/camera_main/camera_info', CameraInfo, self.__callback_info, queue_size=1)

    def __callback_image(self, msg):
        with self.__lock:
            self.__image = msg

    def __callback_info(self, msg):
        with self.__lock:
            self.__info = msg

    def __to_gray(self, msg):
        """Convert a ROS image to a grayscale numpy array (without cv_bridge)."""
        np = self.__np
        cv2 = self.__cv2
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding)
        if channels is None:
            raise ValueError('unsupported encoding %s' % msg.encoding)
        # msg.step = bytes per row (it can be larger than width * channels because of padding)
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = frame[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        if channels == 1:
            return frame[:, :, 0]
        code = {'rgb8': cv2.COLOR_RGB2GRAY, 'bgr8': cv2.COLOR_BGR2GRAY,
                'rgba8': cv2.COLOR_RGBA2GRAY, 'bgra8': cv2.COLOR_BGRA2GRAY}[msg.encoding]
        return cv2.cvtColor(frame, code)

    def measure(self, marker_id):
        """Return a one-line text for the log describing where marker `marker_id` is."""
        if self.__cv2 is None:
            return 'vision unavailable'
        with self.__lock:
            image, info = self.__image, self.__info
        if image is None or info is None:
            return 'no image/camera_info yet'
        try:
            gray = self.__to_gray(image)
            corners, ids, _ = self.__cv2.aruco.detectMarkers(gray, self.__dictionary, parameters=self.__parameters)
            if ids is None:
                return 'marker %d not detected (no markers)' % marker_id
            for marker_corners, detected in zip(corners, ids.flatten()):
                if int(detected) != marker_id:
                    continue
                center = marker_corners.reshape(4, 2).mean(axis=0)      # average of the 4 corners
                # K = [fx 0 cx; 0 fy cy; 0 0 1]: (cx, cy) is the optical center in pixels
                fx, cx, fy, cy = info.K[0], info.K[2], info.K[4], info.K[5]
                dx, dy = center[0] - cx, center[1] - cy
                angle = math.degrees(math.atan(math.hypot(dx / fx, dy / fy)))
                return 'marker %d pixel offset dx=%.1f dy=%.1f (%.2f deg off optical axis)' % (
                    marker_id, dx, dy, angle)
            return 'marker %d not detected (seen %s)' % (marker_id, ids.flatten().tolist())
        except Exception as exc:
            return 'vision error: %s' % exc
