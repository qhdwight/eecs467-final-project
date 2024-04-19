from manifpy import SE2

from geometry_msgs.msg import TransformStamped

import math

def to_se2(t: TransformStamped) -> SE2:
    # Assume the quaternion is normalized and has axis (0, 0, 1)
    angle = math.acos(t.transform.rotation.w) * 2
    return SE2(t.transform.translation.x, t.transform.translation.y, angle)
