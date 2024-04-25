#!/usr/bin/env python3

from hockey_cup.msg import GameState
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
)
from transforms import *

broadcaster = TransformBroadcaster()
UPDATE_RATE = 20
FIELD_LENGTH = 2.44
FIELD_WIDTH = 1.22


def game() -> None:
    rospy.init_node("game")

    game_state_pub = rospy.Publisher("game_state", GameState, queue_size=1)

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    scores = (0, 0)

    rate = rospy.Rate(UPDATE_RATE)
    while not rospy.is_shutdown():
        # Ball speed computation

        BALL_LOOK_BEHIND = 0.5
        try:
            ball_in_map = tf2_buffer.lookup_transform("map", "ball", rospy.Time(0))
            past_time = rospy.Time.now() - rospy.Duration(BALL_LOOK_BEHIND)
            ball_in_map_past = tf2_buffer.lookup_transform("map", "ball", past_time)
        except:
            ball_in_map = None
            ball_in_map_past = None

        if ball_in_map is not None and ball_in_map_past is not None:
            delta = to_se2(ball_in_map).translation() - to_se2(ball_in_map_past).translation()
            ball_speed = np.linalg.norm(delta) / BALL_LOOK_BEHIND
        else:
            ball_speed = np.nan

        # Scoring computation

        if ball_in_map is not None and ball_in_map_past is not None:
            if ball_in_map.transform.translation.x < -FIELD_LENGTH / 2 + 0.1 and ball_in_map_past.transform.translation.x >= -FIELD_LENGTH / 2 + 0.1:
                scores[1] += 1
            if ball_in_map.transform.translation.x > FIELD_LENGTH / 2 - 0.1 and ball_in_map_past.transform.translation.x <= FIELD_LENGTH / 2 - 0.1:
                scores[0] += 1

        game_state_pub.publish(GameState(tuple(scores), ball_speed))

        rate.sleep()


if __name__ == "__main__":
    try:
        game()
    except rospy.ROSInterruptException:
        pass
