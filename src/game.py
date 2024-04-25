#!/usr/bin/env python3
import rospy
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

    scores = [0] * 2

    scoring_on_cooldown = False

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
            rospy.logwarn_throttle(1, "Ball not detected. Going too fast?")

        if ball_in_map is not None and ball_in_map_past is not None:
            delta = to_se2(ball_in_map).translation() - to_se2(ball_in_map_past).translation()
            ball_speed = np.linalg.norm(delta) / BALL_LOOK_BEHIND
        else:
            ball_speed = np.nan

        # Scoring computation

        if ball_in_map is not None and ball_in_map_past is not None:
            if not scoring_on_cooldown:
                def timer_cb(_):
                    nonlocal scoring_on_cooldown
                    scoring_on_cooldown = False

                BUFFER = 0.05

                x, px = ball_in_map.transform.translation.x, ball_in_map_past.transform.translation.x
                if x < -FIELD_LENGTH / 2 + BUFFER <= px:
                    scores[1] += 1
                    scoring_on_cooldown = True
                    rospy.Timer(rospy.Duration(2), timer_cb, oneshot=True)
                if x > FIELD_LENGTH / 2 - BUFFER >= px:
                    scores[0] += 1
                    scoring_on_cooldown = True
                    rospy.Timer(rospy.Duration(2), timer_cb, oneshot=True)

        game_state_pub.publish(GameState(tuple(scores), ball_speed))

        rate.sleep()


if __name__ == "__main__":
    try:
        game()
    except rospy.ROSInterruptException:
        pass
