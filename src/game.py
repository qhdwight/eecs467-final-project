#!/usr/bin/env python3

import rospy
import geometry_msgs.msg

from geometry_msgs.msg import TransformStamped
from hockey_cup.msg import GameState


from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

UPDATE_RATE = 20
SCORES = (0, 0)
FIELD_LENGTH = 2.44
FIELD_WIDTH = 1.22

def publish_goal_pose(ball_in_map: geometry_msgs.msg.TransformStamped) -> None:



def game() -> None:
    rospy.init_node("game")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    game_state_pub = rospy.Publisher("game_state", GameState, queue_size=1)

    rate = rospy.Rate(UPDATE_RATE)

    old_pose = None

    while not rospy.is_shutdown():
        try:
            ball_in_map = tf2_buffer.lookup_transform("map", "ball", rospy.Time(0))
            publish_goal_pose(old_pose, ball_in_map)
        except (LookupException, ConnectivityException, ExtrapolationException):
            ball_in_map = None

        if ball_in_map is None:
            turn = -1
            rospy.logwarn_throttle(1, "Game could not find the ball")
        elif ball_in_map.transform.translation.x > 0:
            turn = 0
        else:
            turn = 1

        game_state_pub.publish(GameState(turn, SCORES))

        rate.sleep()


if __name__ == "__main__":
    try:
        game()
    except rospy.ROSInterruptException:
        pass
