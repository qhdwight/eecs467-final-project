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

from typing import Optional

from enum import Enum

from hockey_cup.msg import BallControl

class PlayerState(Enum):
    BLOCKING = 0
    RETRIEVING = 1
    GOING_TO_SHOOT = 2
    SHOOTING = 3

BALL_CHILL_SPEED = 0.1

TAU = 2 * np.pi

UPDATE_RATE = 20

def player() -> None:
    rospy.init_node("player")

    bot_number = rospy.get_param("~number")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    tf2_broadcaster = TransformBroadcaster()

    def push_goal_to_tf(goal: SE2) -> None:
        tf2_broadcaster.sendTransform(to_tf(goal, "map", f"goal_{bot_number}"))

    def se2_from_tf(name: str) -> Optional[SE2]:
        try:
            return to_se2(tf2_buffer.lookup_transform("map", name, rospy.Time(0)))
        except Exception:
            return None

    initial_pose = None

    last_known_ball_in_map = None

    state = PlayerState.BLOCKING
    prev_state = state

    goal_pose = None

    ball_control_pub = rospy.Publisher(f"cmd_ball_{bot_number}", BallControl, queue_size=1)

    def game_state_callback(game_state: GameState) -> None:
        nonlocal initial_pose, last_known_ball_in_map, prev_state, goal_pose

        ball_in_map = se2_from_tf("ball")
        me_in_map = se2_from_tf(f"bot_{bot_number}")

        if initial_pose is None:
            initial_pose = me_in_map

        ## State Transitions

        is_ball_chilling = np.isfinite(game_state.ball_speed) and game_state.ball_speed < BALL_CHILL_SPEED
        is_ball_on_our_side = ball_in_map is not None and int(ball_in_map.x() < 0) == bot_number
        is_transition = state != prev_state

        if state == PlayerState.RETRIEVING:
            if ball_in_map is None:
                if not is_ball_chilling:
                    state = PlayerState.BLOCKING
            else:
                if last_known_ball_in_map is not None:
                    vx, _, _ = (last_known_ball_in_map - me_in_map).coeffs()
                    we_have_ball = 0 < vx < 0.2
                    if we_have_ball:
                        state = PlayerState.GOING_TO_SHOOT
                else:
                    rospy.logerr("Started with ball covered")
        elif state == PlayerState.BLOCKING:
            if is_ball_on_our_side and is_ball_chilling:
                state = PlayerState.RETRIEVING
        elif state == PlayerState.GOING_TO_SHOOT:
            is_at_goal = np.allclose(me_in_map.translation(), goal_pose.translation(), atol=0.1)
            if is_at_goal:
                state = PlayerState.SHOOTING
        elif state == PlayerState.SHOOTING:
            if ball_in_map is not None:
                state = PlayerState.BLOCKING
            

        ## Goal Evaluation

        if state == PlayerState.BLOCKING:
            # TODO compute trajectory to block the ball
            ...
        elif state == PlayerState.RETRIEVING:
            angle_to_ball = math.atan2(*(ball_in_map.translation() - me_in_map.translation())[::-1])
            goal_pose = SE2(ball_in_map.x(), ball_in_map.y(), angle_to_ball)
            if is_transition:
                ball_control_pub.publish(BallControl(1, 0))
        elif state == PlayerState.GOING_TO_SHOOT:
            if is_transition:
                goal_pose = SE2(0, 0, initial_pose.angle()) + SE2Tangent(
                    np.random.uniform(-0.1, 0.1),
                    np.random.uniform(-0.1, 0.1),
                    np.random.uniform(-TAU / 4, TAU / 4),
                )
        elif state == PlayerState.SHOOTING:
            if is_transition:
                ball_control_pub.publish(BallControl(0, 1))

        if is_transition and prev_state == PlayerState.GOING_TO_SHOOT:
            ball_control_pub.publish(BallControl(0, 0))

        push_goal_to_tf(goal_pose)

        last_known_ball_in_map = ball_in_map
        prev_state = state

    rospy.Subscriber("game_state", GameState, game_state_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        player()
    except rospy.ROSInterruptException:
        pass
