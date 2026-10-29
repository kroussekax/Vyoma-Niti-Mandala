# -*- coding:utf-8 -*-
"""
krpc_geometry.py - 3D math helpers (vectors and rotations).
===========================================================

This file uses only plain Python (no ROS, no numpy), so you can run and test it
on any computer.

Quick refresher
---------------
Vector  [x, y, z]
    An arrow in 3D space. It is used for positions ("where is the robot?") and
    directions ("where is the robot looking?").

Quaternion  [x, y, z, w]
    Four numbers that describe a ROTATION, meaning which way the robot faces.
    You do not need to understand the formula inside. Remember these facts:
      * [0, 0, 0, 1] means "no rotation".
      * q and -q describe the same orientation.
      * The order here is x, y, z, w. This is the same order as ROS and the judge.

Where does the robot look?
    The camera points along the robot's own +X axis ("forward").
    To find where the camera looks in the world, rotate [1, 0, 0] by the
    robot's quaternion:  forward = rotate_vector([1, 0, 0], q)

Roll ("twist")
    Picture the robot looking straight at the marker, then spinning around that
    line of sight like a drill. That spin angle is the roll. The judge wants our
    roll to match a target roll that it chose at random.
"""

import math


# ---------------------------------------------------------------------------
# Vector basics
# ---------------------------------------------------------------------------

def norm(v):
    """Length of a vector (Pythagoras in 3D)."""
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def sub(a, b):
    """a - b. For points, this is the arrow that goes FROM b TO a."""
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def dot(a, b):
    """Dot product. For unit vectors it equals cos(angle between them)."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    """Cross product: a vector perpendicular to both a and b."""
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def distance(a, b):
    """Straight-line distance between two points [m]."""
    return norm(sub(a, b))


def normalize_vector(v, fallback=None):
    """Same direction, length 1. Returns `fallback` (or zeros) for a zero vector."""
    n = norm(v)
    if n <= 1e-9:
        return list(fallback) if fallback is not None else [0.0, 0.0, 0.0]
    return [v[0] / n, v[1] / n, v[2] / n]


def vector_angle_deg(a, b):
    """Angle between two directions, in degrees (0 = same direction)."""
    a = normalize_vector(a)
    b = normalize_vector(b)
    # min/max protects acos() from tiny rounding errors such as 1.0000000002
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(a, b)))))


def wrap_angle_deg(angle_deg):
    """Bring any angle into the range [-180, 180). Example: 350 -> -10."""
    return (angle_deg + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# Quaternion basics
# ---------------------------------------------------------------------------

def quat_normalize(q):
    """Scale a quaternion to length 1 (only unit quaternions are valid rotations)."""
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n <= 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]


def quat_multiply(q1, q2):
    """Combine two rotations: first apply q2, then q1."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def quat_conjugate(q):
    """The opposite rotation (undo q)."""
    return [-q[0], -q[1], -q[2], q[3]]


def quat_from_axis_angle(axis, angle_rad):
    """Rotation of `angle_rad` radians around the direction `axis`."""
    a = normalize_vector(axis, fallback=[1.0, 0.0, 0.0])
    s = math.sin(angle_rad / 2.0)
    return [a[0] * s, a[1] * s, a[2] * s, math.cos(angle_rad / 2.0)]


def rotate_vector(v, q):
    """Rotate vector v by quaternion q (standard formula: v' = q * v * q^-1)."""
    q = quat_normalize(q)
    p = quat_multiply(quat_multiply(q, [v[0], v[1], v[2], 0.0]), quat_conjugate(q))
    return [p[0], p[1], p[2]]


def quat_angle_deg(q1, q2):
    """How many degrees the robot must turn to go from orientation q1 to q2."""
    a = quat_normalize(q1)
    b = quat_normalize(q2)
    # abs() because q and -q are the same orientation
    d = abs(max(-1.0, min(1.0, a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])))
    return math.degrees(2.0 * math.acos(d))


def quat_from_matrix_columns(x_axis, y_axis, z_axis):
    """Quaternion for a rotation whose new X, Y, Z axes are the given unit vectors.
    (Standard "rotation matrix -> quaternion" conversion; the 4 branches avoid
    dividing by a number close to zero.)"""
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = [(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s]
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = [0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s]
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = [(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s]
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = [(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s]
    return quat_normalize(q)


# ---------------------------------------------------------------------------
# "Look at" helpers - point the camera at a marker
# ---------------------------------------------------------------------------

def look_at_quaternion(forward):
    """An orientation whose camera (+X axis) points along `forward`.
    It is built the same way as the simulator's checkpoint generator:
      X = forward, Z = X x up, Y = Z x X  (three perpendicular axes)."""
    x_axis = normalize_vector(forward, fallback=[1.0, 0.0, 0.0])
    up = [0.0, 0.0, 1.0]
    if abs(dot(x_axis, up)) > 0.95:          # looking almost straight up/down:
        up = [0.0, 1.0, 0.0]                 # pick another "up" to avoid a zero cross product
    z_axis = normalize_vector(cross(x_axis, up), fallback=[0.0, 0.0, 1.0])
    y_axis = normalize_vector(cross(z_axis, x_axis), fallback=[0.0, 1.0, 0.0])
    return quat_from_matrix_columns(x_axis, y_axis, z_axis)


def twist_angle_deg(q, axis):
    """Roll of orientation q around `axis`, in degrees (with sign).
    This is copied exactly from the judge, so our roll check matches its check."""
    qx, qy, qz, qw = quat_normalize(q)
    if qw < 0.0:                             # use the "w >= 0" version of q (same orientation)
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    return math.degrees(2.0 * math.atan2(qx * axis[0] + qy * axis[1] + qz * axis[2], qw))


def look_at_with_roll(position, marker_position, reference_quaternion):
    """Orientation that, from `position`, looks exactly at the marker AND has the
    same roll as `reference_quaternion` (the judge's target attitude).

    Used for corrections: if the robot stopped a few cm away from the checkpoint,
    aim the camera from where it really is, so the marker stays centered."""
    axis = normalize_vector(sub(marker_position, position), fallback=[1.0, 0.0, 0.0])
    base = look_at_quaternion(axis)
    # Spin `base` around the line of sight until its roll equals the reference roll.
    delta_deg = twist_angle_deg(reference_quaternion, axis) - twist_angle_deg(base, axis)
    roll = quat_from_axis_angle(axis, math.radians(wrap_angle_deg(delta_deg)))
    return quat_normalize(quat_multiply(roll, base))


def centering_and_roll_deg(position, quaternion, marker_position, target_quaternion):
    """The judge's two attitude scores (copy of judge.py _evaluate_centering_and_roll).

    centering: angle between "where the camera looks" and "where the marker is",
               measured from the robot CENTER. 0 deg = marker exactly in the middle.
    roll:      difference between our roll and the target roll around the line
               of sight. 0 deg = perfect.
    """
    view_axis = normalize_vector(sub(marker_position, position))            # robot -> marker
    camera_forward = normalize_vector(rotate_vector([1.0, 0.0, 0.0], quaternion))
    centering = math.degrees(math.acos(max(-1.0, min(1.0, dot(camera_forward, view_axis)))))
    roll_actual = twist_angle_deg(quaternion, view_axis)
    roll_target = twist_angle_deg(target_quaternion, view_axis)
    return centering, abs(wrap_angle_deg(roll_actual - roll_target))
