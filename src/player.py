#!/usr/bin/env python3

import math

from hockey_cup.msg import GameState

import rospy
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
)

UPDATE_RATE = 20


def player() -> None:
    rospy.init_node("game")

    number = rospy.get_param("~number")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    tf2_broadcaster = TransformBroadcaster()

    def push_goal_to_tf(x: float, y: float, angle: float) -> None:
        tf2_broadcaster.sendTransform(TransformStamped(
            header=rospy.Header(
                stamp=rospy.Time.now(),
                frame_id="map",
            ),
            child_frame_id=f"goal_{number}",
            transform=Transform(
                translation=Vector3(x, y, 0),
                rotation=Quaternion(0, 0, math.sin(angle / 2), math.cos(angle / 2))
            )
        ))

    def game_state_callback(game_state: GameState) -> None:
        if game_state.turn == number:
            try:
                ball_in_map = tf2_buffer.lookup_transform("map", "ball", rospy.Time(0))
                me_in_map = tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0))

                angle_to_ball = math.atan2(
                    ball_in_map.transform.translation.y - me_in_map.transform.translation.y,
                    ball_in_map.transform.translation.x - me_in_map.transform.translation.x,
                )

                push_goal_to_tf(ball_in_map.transform.translation.x, ball_in_map.transform.translation.y, angle_to_ball)
            except Exception as e:
                rospy.logwarn_throttle(1, f"Player failed: {e}")
        else:
            push_goal_to_tf(-2, 2, 3.14)

    rospy.Subscriber("game_state", GameState, game_state_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        player()
    except rospy.ROSInterruptException:
        pass
