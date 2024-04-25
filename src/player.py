#!/usr/bin/env python3

import math

from hockey_cup.msg import GameState

import rospy
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
)

from manifpy import SE2

from transforms import *

UPDATE_RATE = 20


def player() -> None:
    rospy.init_node("player")

    number = rospy.get_param("~number")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    tf2_broadcaster = TransformBroadcaster()

    def push_goal_to_tf(goal: SE2) -> None:
        tf2_broadcaster.sendTransform(to_tf(goal, "map", f"goal_{number}"))

    initial_pose = None

    def game_state_callback(game_state: GameState) -> None:
        try:
            ball_in_map = to_se2(
                tf2_buffer.lookup_transform("map", "ball", rospy.Time(0))
            )
            me_in_map = to_se2(
                tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0))
            )

            nonlocal initial_pose
            if initial_pose is None:
                initial_pose = me_in_map

            if game_state.turn == number:
                angle_to_ball = math.atan2(*(ball_in_map.translation() - me_in_map.translation())[::-1])
                push_goal_to_tf(SE2(ball_in_map.x(), ball_in_map.y(), angle_to_ball))
            else:
                push_goal_to_tf(initial_pose)

        except Exception as e:
            rospy.logwarn_throttle(1, f"Player failed: {e}")

    rospy.Subscriber("game_state", GameState, game_state_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        player()
    except rospy.ROSInterruptException:
        pass
