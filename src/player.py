#!/usr/bin/env python3

import rospy

from hockey_cup.msg import GameState
from hockey_cup.msg import FollowPathAction

from tf2_ros import Buffer, TransformListener

UPDATE_RATE = 20

def player() -> None:
    rospy.init_node("game")

    player_number = rospy.get_param("~player_number")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    def game_state_callback(game_state: GameState) -> None:
        if game_state.turn == player_number:
            ball_in_map = tf2_buffer.lookup_transform("map", "ball", rospy.Time(0))
            me_in_map = tf2_buffer.lookup_transform("map", f"bot_{player_number}", rospy.Time(0))
        else:
            ...

    game_state_sub = rospy.Subscriber("game_state", GameState, game_state_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        player()
    except rospy.ROSInterruptException:
        pass
