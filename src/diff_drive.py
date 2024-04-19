#!/usr/bin/env python3

import numpy as np

import rospy
from geometry_msgs.msg import Twist

from hockey_cup.msg import WheelVelocities

WHEEL_DISTANCE = 0.15
WHEEL_RADIUS = 0.042


def diff_drive() -> None:
    rospy.init_node('diff_drive')

    number = rospy.get_param('~number', 0)
    cmd_wheel_vels_topic = f'cmd_wheel_vels_{number}'
    cmd_wheel_vels_pub = rospy.Publisher(cmd_wheel_vels_topic, WheelVelocities, queue_size=1)

    def twist_callback(message: Twist) -> None:
        wheel_speeds = np.array([
            [1, -WHEEL_DISTANCE / 2],
            [1, WHEEL_DISTANCE / 2],
        ]) @ np.array([message.linear.x, message.angular.z]) / WHEEL_RADIUS
        cmd_wheel_vels_pub.publish(WheelVelocities(*wheel_speeds))

    cmd_vel_topic = f'cmd_vel_{number}'
    rospy.Subscriber(cmd_vel_topic, Twist, twist_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        diff_drive()
    except rospy.ROSInterruptException:
        pass
