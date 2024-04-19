#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Vector3

from transforms import to_se2

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

import numpy as np


def motion_planner() -> None:
    rospy.init_node("motion_planner")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    number = rospy.get_param('~number', 0)
    cmd_vel_topic = f'cmd_vel_{number}'
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            goal_in_map = to_se2(tf2_buffer.lookup_transform("map", f"goal_{number}", rospy.Time(0)))
            bot_in_map = to_se2(tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0)))

            # distance = np.linalg.norm([goal_in_map.x() - bot_in_map.x(), goal_in_map.y() - bot_in_map.y()])
            # if distance < 0.1:
            #     cmd_vel_pub.publish(Twist())
            #     rate.sleep()
            #     continue

            K1 = 0.3
            K2 = 1.4
            K3 = 1.0

            # Motion planning is based off section 13.3.4 in "Modern Robotics"

            # Equation 13.30
            b = np.array([bot_in_map.x(), bot_in_map.y(), bot_in_map.angle()])
            g = np.array([goal_in_map.x(), goal_in_map.y(), goal_in_map.angle()])
            xe, ye, pe = np.array([
                [np.cos(g[2]), np.sin(g[2]), 0],
                [-np.sin(g[2]), np.cos(g[2]), 0],
                [0, 0, 1],
            ]) @ (b - g)

            vd = 1
            wd = 0

            # Equation 13.31
            v = (vd - K1 * abs(vd) * (xe + ye * np.tan(pe))) / np.cos(pe)
            w = wd - (K2 * vd * ye + K3 * abs(vd) * np.tan(pe)) * np.cos(pe) ** 2

            # v *= -1
            # w *= -1

            v = np.clip(v, -1, 1)
            w = np.clip(w, -1, 1)


            cmd_vel_pub.publish(Twist(
                linear=Vector3(x=v),
                angular=Vector3(z=w),
            ))

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        rate.sleep()


if __name__ == "__main__":
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
