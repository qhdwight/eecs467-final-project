#!/usr/bin/env python3

from enum import Enum
from typing import Optional, Tuple

from manifpy import SE2Tangent

from hockey_cup.msg import BallControl
from hockey_cup.msg import GameState
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from transforms import *


class PlayerState(Enum):
    BLOCKING = 0
    RETRIEVING = 1
    GOING_TO_SHOOT = 2
    SHOOTING = 3


BALL_CHILL_SPEED = 0.1

TAU = 2 * np.pi

FIELD_LENGTH = 2.44
FIELD_WIDTH = 1.22
LATERAL_DEFENSE_LINE = 0.75


def player() -> None:
    rospy.init_node("player")

    bot_number = rospy.get_param("~number", 0)

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    tf2_broadcaster = TransformBroadcaster()

    def push_goal_to_tf(goal: SE2) -> None:
        tf2_broadcaster.sendTransform(to_tf(goal, "map", f"goal_{bot_number}"))

    def se2_from_tf_timed(name: str, time: rospy.Time) -> Tuple[Optional[SE2], rospy.Time]:
        try:
            tf = tf2_buffer.lookup_transform("map", name, time)
            return to_se2(tf), tf.header.stamp
        except:
            return None, time

    def se2_from_tf(name: str) -> Tuple[Optional[SE2], rospy.Time]:
        return se2_from_tf_timed(name, rospy.Time(0))

    state = PlayerState.BLOCKING
    prev_state = state

    goal_pose = None

    ball_control_pub = rospy.Publisher(f"cmd_ball_{bot_number}", BallControl, queue_size=1)

    global LATERAL_DEFENSE_LINE
    lateral_defense_line = LATERAL_DEFENSE_LINE if bot_number == 0 else -LATERAL_DEFENSE_LINE
    last_good_y_pose = 0

    time_at_goal_ticks = 0

    def compute_block_pose(ball_in_map) -> SE2:
        nonlocal lateral_defense_line, last_good_y_pose

        block_pose = None

        ball_in_map_prev, _ = se2_from_tf_timed("ball", rospy.Time.now() - rospy.Duration(0.5))
        if ball_in_map_prev is not None:
            # If the ball is not moving sufficiently quickly in the x direction, just publish the last good y pose
            dx, dy = ball_in_map.translation() - ball_in_map_prev.translation()
            if abs(dx) > 0.1:
                proposed_goal_pose_y = (lateral_defense_line - ball_in_map_prev.x()) * dy / dx + ball_in_map_prev.y()
                # Simulate bouncing off walls
                for i in range(3):
                    if proposed_goal_pose_y < -FIELD_WIDTH / 2:
                        proposed_goal_pose_y = -FIELD_WIDTH - proposed_goal_pose_y
                    elif proposed_goal_pose_y > FIELD_WIDTH / 2:
                        proposed_goal_pose_y = FIELD_WIDTH - proposed_goal_pose_y
                    else:
                        last_good_y_pose = proposed_goal_pose_y
                        break

                # If still out of bounds, publish the last good pose
                if proposed_goal_pose_y < -FIELD_WIDTH / 2 or proposed_goal_pose_y > FIELD_WIDTH / 2:
                    block_pose = SE2(lateral_defense_line, proposed_goal_pose_y, 0)

        if block_pose is None:
            block_pose = SE2(lateral_defense_line, last_good_y_pose, TAU / 4)

        return block_pose

    def game_state_callback(game_state: GameState) -> None:
        nonlocal prev_state, goal_pose, state, time_at_goal_ticks

        ball_in_map, ball_timestamp = se2_from_tf("ball")
        is_ball_old = ball_in_map is None or ball_timestamp < rospy.Time.now() - rospy.Duration(0.2)
        bot_in_map, _ = se2_from_tf(f"bot_{bot_number}")

        # State Transitions

        is_ball_chilling = np.isfinite(game_state.ball_speed) and game_state.ball_speed < BALL_CHILL_SPEED
        is_ball_on_our_side = ball_in_map is not None and int(ball_in_map.x() < 0) == bot_number
        is_ball_stowed_or_close = is_ball_old or (
                bot_in_map is not None and np.linalg.norm(ball_in_map.translation() - bot_in_map.translation()) < 0.2
        )

        if state == PlayerState.RETRIEVING:
            if is_ball_stowed_or_close:
                state = PlayerState.GOING_TO_SHOOT
            if not is_ball_on_our_side:
                rospy.logwarn("Ball is not on our side anymore")
                state = PlayerState.BLOCKING
        elif state == PlayerState.BLOCKING:
            if is_ball_on_our_side and is_ball_chilling:
                state = PlayerState.RETRIEVING
        elif state == PlayerState.GOING_TO_SHOOT:
            if is_ball_stowed_or_close:
                is_at_goal = se2_within(goal_pose, bot_in_map)
                if is_at_goal:
                    time_at_goal_ticks += 1
                    if time_at_goal_ticks > 80:
                        state = PlayerState.SHOOTING
                        time_at_goal_ticks = 0
            else:
                rospy.logwarn(f"Ball popped out while going to shoot")
                if is_ball_on_our_side:
                    state = PlayerState.RETRIEVING
                else:
                    state = PlayerState.BLOCKING
        elif state == PlayerState.SHOOTING:
            if not is_ball_stowed_or_close:
                state = PlayerState.BLOCKING

        is_transition = state != prev_state
        if is_transition:
            rospy.loginfo(f"Transitioning from {prev_state} => {state}")

        # Goal Evaluation

        if state == PlayerState.BLOCKING:
            # TODO compute trajectory to block the ball
            goal_pose = compute_block_pose(ball_in_map)
            ball_control_pub.publish(BallControl(0, 0))
        elif state == PlayerState.RETRIEVING:
            angle_to_ball = math.atan2(*(ball_in_map.translation() - bot_in_map.translation())[::-1])
            goal_pose = SE2(ball_in_map.x(), ball_in_map.y(), angle_to_ball)
            ball_control_pub.publish(BallControl(1, 0))
        elif state == PlayerState.GOING_TO_SHOOT:
            if is_transition:
                goal_pose = SE2(bot_in_map.x(), bot_in_map.y(), np.pi if bot_number == 0 else 0) + SE2Tangent(
                    0, 0,
                    np.random.uniform(-TAU / 8, TAU / 8),
                )
            ball_control_pub.publish(BallControl(1, 0))
        elif state == PlayerState.SHOOTING:
            if is_transition:
                ball_control_pub.publish(BallControl(0, 1))
                rospy.sleep(1)

        push_goal_to_tf(goal_pose)

        prev_state = state

    rospy.Subscriber("game_state", GameState, game_state_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        player()
    except rospy.ROSInterruptException:
        pass
