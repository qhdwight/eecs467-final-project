from manifpy import SE2

from geometry_msgs.msg import TransformStamped

import math

def to_se2(t: TransformStamped) -> SE2:
    # Quaternions are fundamentally angle-axis representations
    # We have some what of a restricted case since we're only rotating about the z-axis, therefore:
    # z = sin(theta / 2)
    # w = cos(theta / 2)
    # It follows that theta = 2 * arccos(w)
    # But the range of arccos is [0, pi], so we need to add the sign of z to get the full range of rotation
    angle = math.copysign(2 * math.acos(t.transform.rotation.w), t.transform.rotation.z)
    return SE2(t.transform.translation.x, t.transform.translation.y, angle)
