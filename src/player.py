#!/usr/bin/env python3

import rospy

from hockey_cup.msg import GameState

from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from geometry_msgs.msg import Pose2D

from transforms import to_se2

UPDATE_RATE = 20


def player() -> None:
    rospy.init_node("game")

    number = rospy.get_param("~number")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    goal_pose_publisher = rospy.Publisher(f"goal_pose_{number}", Pose2D, queue_size=1)

    def game_state_callback(game_state: GameState) -> None:
        if game_state.turn == number:
            try:
                ball_in_map = tf2_buffer.lookup_transform("map", "ball", rospy.Time(0))
                me_in_map = tf2_buffer.lookup_transform(
                    "map", f"bot_{number}", rospy.Time(0)
                )

                goal_pose_publisher.publish(
                    Pose2D(
                        x=ball_in_map.transform.translation.x,
                        y=ball_in_map.transform.translation.y,
                        theta=(to_se2(ball_in_map) - to_se2(me_in_map)).angle(),
                    )
                )
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
