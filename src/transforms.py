import math

import numpy as np
from manifpy import SE2

from numpy.typing import NDArray

import rospy
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion


def to_se2(t: TransformStamped) -> SE2:
    # Quaternions are fundamentally angle-axis representations
    # We have somewhat of a restricted case since we're only rotating about the z-axis (0, 0, 1), therefore:
    #
    # x ~= 0
    # y ~= 0
    # z = sin(theta / 2)
    # w = cos(theta / 2)
    #
    # It follows that theta = 2 * acos(w)
    # But the range of acos is [0, pi], so we need to add the sign of z to get the full range of rotation
    angle = math.copysign(2 * math.acos(t.transform.rotation.w), t.transform.rotation.z)
    return SE2(t.transform.translation.x, t.transform.translation.y, angle)


def to_tf(se2: SE2, parent_frame: str, child_frame: str) -> TransformStamped:
    return TransformStamped(
        header=rospy.Header(
            stamp=rospy.Time.now(),
            frame_id=parent_frame
        ),
        child_frame_id=child_frame,
        transform=Transform(
            translation=Vector3(x=se2.x(), y=se2.y()),
            rotation=Quaternion(z=math.sin(se2.angle() / 2), w=math.cos(se2.angle() / 2))
        )
    )


def angle_to(from_se2: SE2, to_se2: SE2) -> float:
    direction = to_se2.translation() - from_se2.translation()
    angle_to_goal = math.atan2(*direction[::-1]) - from_se2.angle()
    return (angle_to_goal + math.pi) % (2 * math.pi) - math.pi


def se2_to_np(se2: SE2) -> NDArray[np.float64]:
    return np.array([se2.x(), se2.y(), se2.angle()])


def se2_distance(a: SE2, b: SE2) -> float:
    return np.linalg.norm((a - b).coeffs(), ord=2)
