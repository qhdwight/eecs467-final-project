#!/usr/bin/env python3

import numpy as np

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from transforms import *

from math import copysign


def motion_planner() -> None:
    rospy.init_node("motion_planner")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    number = rospy.get_param('~number', 0)
    cmd_vel_topic = f'cmd_vel_{number}'
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    def control_law(bot_in_map: SE2, goal_in_map: SE2) -> Twist:
        tangent = goal_in_map - bot_in_map
        tangent = tangent.coeffs()
        tangent *= [2, 12, 5]
        tangent[2] += copysign(tangent[1], tangent[0])
        tangent[1] = 0
        tangent = np.clip(tangent, [-0.6, -0.6, -np.pi], [0.6, 0.6, np.pi])
        v, _, w = tangent
        return Twist(
            linear=Vector3(x=v),
            angular=Vector3(z=w)
        )

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            goal_in_map = to_se2(tf2_buffer.lookup_transform("map", f"goal_{number}", rospy.Time(0)))
            bot_in_map = to_se2(tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0)))

            twist = control_law(bot_in_map, goal_in_map)
            cmd_vel_pub.publish(twist)

        except Exception as e:
            rospy.logwarn_throttle(1, f"Motion planner error: {e}")

        rate.sleep()


if __name__ == "__main__":
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
