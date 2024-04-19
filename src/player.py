#!/usr/bin/env python3

import rospy

from hockey_cup.msg import GameState

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
    TransformBroadcaster,
)

from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion

from transforms import to_se2

import math

UPDATE_RATE = 20


def player() -> None:
    rospy.init_node("game")

    number = rospy.get_param("~number")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    tf2_broadcaster = TransformBroadcaster()

    def game_state_callback(game_state: GameState) -> None:
        if game_state.turn == number:
            try:
                ball_in_map = tf2_buffer.lookup_transform("map", "ball", rospy.Time(0))
                me_in_map = tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0))

                angle_to_ball = math.atan2(
                    ball_in_map.transform.translation.y - me_in_map.transform.translation.y,
                    ball_in_map.transform.translation.x - me_in_map.transform.translation.x,
                )
                
                tf2_broadcaster.sendTransform(TransformStamped(
                    header=rospy.Header(
                        stamp=rospy.Time.now(),
                        frame_id="map",
                    ),
                    child_frame_id=f"goal_{number}",
                    transform=Transform(
                        translation = ball_in_map.transform.translation,
                        rotation = Quaternion(0, 0, math.sin(angle_to_ball / 2), math.cos(angle_to_ball / 2)
                    )
                )))
            except (LookupException, ConnectivityException, ExtrapolationException):
                rospy.logwarn("Ball not found")
        else:
            ...

    game_state_sub = rospy.Subscriber("game_state", GameState, game_state_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        player()
    except rospy.ROSInterruptException:
        pass
