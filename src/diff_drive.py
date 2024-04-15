#!/usr/bin/env python3

import numpy as np

import rospy
from geometry_msgs.msg import Twist

from hockey_cup.msg import WheelVelocities


def diff_drive() -> None:
    rospy.init_node('diff_drive')

    cmd_wheel_vels_pub = rospy.Publisher('cmd_wheel_vels', WheelVelocities, queue_size=1)

    wheel_distance = 0.15

    def twist_callback(message: Twist) -> None:
        left_wheel, right_wheel = np.array([
            [1, -wheel_distance / 2],
            [1, wheel_distance / 2],
        ]) @ np.array([message.linear.x, message.angular.z])
        cmd_wheel_vels_pub.publish(WheelVelocities(left_wheel, right_wheel))

    rospy.Subscriber('cmd_vel', Twist, twist_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        diff_drive()
    except rospy.ROSInterruptException:
        pass
