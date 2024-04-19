#!/usr/bin/env python3

import rospy

from hockey_cup.msg import FollowPathAction
from geometry_msgs.msg import Twist

import actionlib


def motion_planner() -> None:
    rospy.init_node("motion_planner")

    number = rospy.get_param('~number', 0)
    cmd_vel_topic = f'cmd_vel_{number}'
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    server = None

    def follow_path_callback(goal: FollowPathAction) -> None:
        # TODO: RTR controller
        ...

    server = actionlib.SimpleActionServer("follow_path", FollowPathAction, follow_path_callback, auto_start=False)
    server.start()

    rospy.spin()


if __name__ == "__main__":
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
