#!/usr/bin/env python3

from enum import Enum
from typing import Optional

from manifpy import SE2, SE2Tangent

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

UPDATE_RATE = 20

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

    def se2_from_tf_timed(name: str, time: rospy.Time) -> Optional[SE2]:
        try:
            return to_se2(tf2_buffer.lookup_transform("map", name, time))
        except:
            return None

    def se2_from_tf(name: str) -> Optional[SE2]:
        return se2_from_tf_timed(name, rospy.Time(0))

    initial_pose = None

    last_known_ball_in_map = None

    state = PlayerState.BLOCKING
    prev_state = state

    goal_pose = None

    ball_control_pub = rospy.Publisher(f"cmd_ball_{bot_number}", BallControl, queue_size=1)

    global LATERAL_DEFENSE_LINE
    lateral_defense_line = LATERAL_DEFENSE_LINE if bot_number == 0 else -LATERAL_DEFENSE_LINE
    last_good_y_pose = 0

    def compute_block_pose(ball_in_map) -> SE2:
        nonlocal lateral_defense_line, last_good_y_pose

        block_pose = None

        ball_in_map_prev = se2_from_tf_timed("ball", rospy.Time.now() - rospy.Duration(0.5))
        if ball_in_map_prev is not None:
            # If the ball is not moving sufficiently quickly in the x direction, just publish the last good y pose
            dx, dy = ball_in_map.translation() - ball_in_map_prev.translation()
            if abs(dx) > 0.1:
                proposed_goal_pose_y = (lateral_defense_line - ball_in_map_prev.x()) * dy / dx + ball_in_map_prev.y()
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
        nonlocal initial_pose, last_known_ball_in_map, prev_state, goal_pose, state

        ball_in_map = se2_from_tf("ball")
        bot_in_map = se2_from_tf(f"bot_{bot_number}")

        if initial_pose is None:
            initial_pose = bot_in_map

        # State Transitions

        is_ball_chilling = np.isfinite(game_state.ball_speed) and game_state.ball_speed < BALL_CHILL_SPEED
        is_ball_on_our_side = ball_in_map is not None and int(ball_in_map.x() < 0) == bot_number

        if state == PlayerState.RETRIEVING:
            if ball_in_map is None:
                # Some bot has the ball
                if last_known_ball_in_map is not None:
                    vx, _, _ = (last_known_ball_in_map - bot_in_map).coeffs()
                    we_have_ball = 0 < vx < 0.2
                    if we_have_ball:
                        rospy.loginfo("We retrieved the ball!")
                        state = PlayerState.GOING_TO_SHOOT
                    else:
                        rospy.loginfo("Retrieval failed and we lost the ball")
                        state = PlayerState.BLOCKING
                else:
                    rospy.logerr("Started with ball covered")
        elif state == PlayerState.BLOCKING:
            if is_ball_on_our_side and is_ball_chilling:
                state = PlayerState.RETRIEVING
        elif state == PlayerState.GOING_TO_SHOOT:
            if ball_in_map is None:
                is_at_goal = np.allclose(bot_in_map.translation(), goal_pose.translation(), atol=0.1)
                if is_at_goal:
                    state = PlayerState.SHOOTING
            else:
                rospy.loginfo("Ball popped out while going to shoot")
                if is_ball_on_our_side:
                    state = PlayerState.RETRIEVING
                else:
                    state = PlayerState.BLOCKING
        elif state == PlayerState.SHOOTING:
            if ball_in_map is not None:
                state = PlayerState.BLOCKING

        is_transition = state != prev_state
        if is_transition:
            rospy.loginfo(f"Transitioning from {prev_state} => {state}")

        # Goal Evaluation

        if state == PlayerState.BLOCKING:
            # TODO compute trajectory to block the ball
            goal_pose = compute_block_pose(ball_in_map)
        elif state == PlayerState.RETRIEVING:
            angle_to_ball = math.atan2(*(ball_in_map.translation() - bot_in_map.translation())[::-1])
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
