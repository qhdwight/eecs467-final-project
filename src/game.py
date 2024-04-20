#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import math

from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import tf2_ros
from hockey_cup.msg import GameState


from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

broadcaster = TransformBroadcaster()
UPDATE_RATE = 20
SCORES = (0, 0)
FIELD_LENGTH = 2.44
FIELD_WIDTH = 1.22
LATERAL_DEFENSE_LINE = 0.75

def push_goal_to_tf(x: float, y: float, angle: float, name: str) -> None:
    tf2_ros.TransformBroadcaster.sendTransform(TransformStamped(
        header=rospy.Header(
            stamp=rospy.Time.now(),
            frame_id="map",
        ),
        child_frame_id=f"goal_{name}",
        transform=Transform(
            translation=Vector3(x, y, 0),
            rotation=Quaternion(0, 0, math.sin(angle / 2), math.cos(angle / 2))
        )
    ))

def publish_goal_pose(prev_ball_pose, ball_in_map: geometry_msgs.msg.TransformStamped) -> None:
    if prev_ball_pose is None or ball_in_map is None:
        return
    
    # calculate pose of ball intersected with the lateral defense line
    pose_prev = (prev_ball_pose.transform.translation.x, prev_ball_pose.transform.translation.y)
    pose_curr = (ball_in_map.transform.translation.x, ball_in_map.transform.translation.y)
    
    # Calculate the pose of the ball along the defense line
    try:
        proposed_goal_pose_y = pose_curr[1] - pose_prev[1] * (LATERAL_DEFENSE_LINE - pose_prev[0]) / (pose_curr[0] - pose_prev[0]) - pose_prev[0]
        # TODO Find appropriate angle
        push_goal_to_tf(LATERAL_DEFENSE_LINE, proposed_goal_pose_y, 0, "defense")
    except:
        print("calculate proposed goal pose failed")
        pass

    # Calculate proposed goal angle


    # If the ball is outside the field, calculate the pose of the ball after an elastic collision with the wall
    # for i in range(3):
    #     if proposed_goal_pose_y < -FIELD_WIDTH / 2:
    #         proposed_goal_pose_y = proposed_goal_pose_y - FIELD_WIDTH / 2
    #     elif proposed_goal_pose_y > FIELD_WIDTH / 2:

    #     else:
    #         break
    #     for i in range(2):
    #         proposed_goal_pose_y = FIELD_WIDTH / 2 if proposed_goal_pose_y > FIELD_WIDTH / 2 else -FIELD_WIDTH / 2
    #         proposed_goal_pose_x = pose_prev[0] + (proposed_goal_pose_y - pose_prev[1]) * (pose_curr[0] - pose_prev[0]) / (pose_curr[1] - pose_prev[1])
    #         if proposed_goal_pose_x > -FIELD_LENGTH / 2 and proposed_goal_pose_x < FIELD_LENGTH / 2:
    #             break
    #         proposed_goal_pose_y = pose_curr[1] - pose_prev[1] * (LATERAL_DEFENSE_LINE - pose_prev[0]) / (pose_curr[0] - pose_prev[0]) - pose_prev[0]

def game() -> None:
    rospy.init_node("game")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    game_state_pub = rospy.Publisher("game_state", GameState, queue_size=1)

    rate = rospy.Rate(UPDATE_RATE)

    prev_ball_pose = None
    ball_in_map = None

    while not rospy.is_shutdown():
        try:
            prev_ball_pose = ball_in_map
            ball_in_map = tf2_buffer.lookup_transform("map", "ball_frame", rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            ball_in_map = None

        if ball_in_map is None:
            turn = -1
            rospy.logwarn_throttle(1, "Ball not found")
        elif ball_in_map.transform.translation.x > 0:
            turn = 0
        else:
            turn = 1
            LATERAL_DEFENSE_LINE = -0.75
        publish_goal_pose(prev_ball_pose, ball_in_map)
        game_state_pub.publish(GameState(turn, SCORES))

        rate.sleep()


if __name__ == "__main__":
    try:
        game()
    except rospy.ROSInterruptException:
        pass
