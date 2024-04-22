#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


def diff_drive() -> None:
    rospy.init_node("diff_drive")

    number = rospy.get_param("~number", 0)

    def twist_callback(message: Twist) -> None:
        ...
        # TODO: Talk to pico with pyserial

    cmd_vel_topic = f"cmd_vel_{number}"
    rospy.Subscriber(cmd_vel_topic, Twist, twist_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        diff_drive()
    except rospy.ROSInterruptException:
        pass
