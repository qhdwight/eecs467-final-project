#!/usr/bin/env python3

import rospy

from hockey_cup.msg import GameState

UPDATE_RATE = 20

def player() -> None:
    rospy.init_node("game")

    player_number = rospy.get_param("~player_number")

    def game_state_callback(game_state: GameState) -> None:
        if game_state.turn == player_number:
            print("My turn!")
        else:
            print("Not my turn...")

    game_state_sub = rospy.Subscriber("game_state", GameState, game_state_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        player()
    except rospy.ROSInterruptException:
        pass
