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
SCORES = (0, 0)
FIELD_LENGTH = 2.44
FIELD_WIDTH = 1.22
LATERAL_DEFENSE_LINE = 0.75
LAST_GOOD_Y_POSE = 0


# def push_goal_to_tf(x: float, y: float, angle: float, name: str) -> None:
#     tf2_ros.TransformBroadcaster.sendTransform(broadcaster, TransformStamped(
#         header=rospy.Header(
#             stamp=rospy.Time.now(),
#             frame_id="map",
#         ),
#         child_frame_id=f"goal_{name}",
#         transform=Transform(
#             translation=Vector3(x, y, 0),
#             rotation=Quaternion(0, 0, math.sin(angle / 2), math.cos(angle / 2))
#         )
#     ))
#
#
# def publish_goal_pose(tf2_buffer, ball_in_map: geometry_msgs.msg.TransformStamped) -> None:
#     if ball_in_map is None:
#         return
#     global LAST_GOOD_Y_POSE
#
#     try:
#         # past_time = rospy.Time.now() - rospy.Duration(0.5)
#         ball_in_map_prev = tf2_buffer.lookup_transform("map", "ball_frame", rospy.Time.now() - rospy.Duration(0.5))
#         # calculate pose of ball intersected with the lateral defense line
#         pose_prev = (ball_in_map_prev.transform.translation.x, ball_in_map_prev.transform.translation.y)
#         pose_curr = (ball_in_map.transform.translation.x, ball_in_map.transform.translation.y)
#
#         # if the ball is not moving sufficiently quickly in the x direction, just publish the last good y pose
#         if abs(pose_curr[0] - pose_prev[0]) < 0.1:
#             push_goal_to_tf(LATERAL_DEFENSE_LINE, LAST_GOOD_Y_POSE, 0, "defense")
#             return
#         # Calculate the pose of the ball along the defense line
#         proposed_goal_pose_y = (LATERAL_DEFENSE_LINE - pose_prev[0]) * (pose_curr[1] - pose_prev[1]) / (
#                 pose_curr[0] - pose_prev[0]) + pose_prev[1]
#
#     except (LookupException, tf2_ros.ExtrapolationException):
#         rospy.loginfo("calculate proposed goal pose failed")
#         proposed_goal_pose_y = LAST_GOOD_Y_POSE
#     except(ZeroDivisionError):
#         rospy.loginfo("ball is stationary")
#         proposed_goal_pose_y = LAST_GOOD_Y_POSE
#     # If the ball is outside the field, calculate the pose of the ball after an elastic collision with the wall
#     for i in range(3):
#         if proposed_goal_pose_y < -FIELD_WIDTH / 2:
#             proposed_goal_pose_y = -FIELD_WIDTH - proposed_goal_pose_y
#         elif proposed_goal_pose_y > FIELD_WIDTH / 2:
#             proposed_goal_pose_y = FIELD_WIDTH - proposed_goal_pose_y
#         else:
#             LAST_GOOD_Y_POSE = proposed_goal_pose_y
#             break
#         # TODO Find appropriate angle
#     # if still out of bounds, publish the last good pose
#     if proposed_goal_pose_y < -FIELD_WIDTH / 2 or proposed_goal_pose_y > FIELD_WIDTH / 2:
#         proposed_goal_pose_y = LAST_GOOD_Y_POSE
#     push_goal_to_tf(LATERAL_DEFENSE_LINE, proposed_goal_pose_y, 0, "defense")


def game() -> None:
    rospy.init_node("game")
    game_state_pub = rospy.Publisher("game_state", GameState, queue_size=1)
    rate = rospy.Rate(UPDATE_RATE)
    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)
    global LATERAL_DEFENSE_LINE

    while not rospy.is_shutdown():
        BALL_LOOK_BEHIND = 0.5
        try:
            ball_in_map = tf2_buffer.lookup_transform("map", "ball", rospy.Time(0))
            ball_in_map_past = tf2_buffer.lookup_transform(
                "map", "ball", rospy.Time.now() - rospy.Duration(BALL_LOOK_BEHIND)
            )
        except:
            ball_in_map = None
            ball_in_map_past = None

        # if ball_in_map is None:
        #     rospy.logwarn_throttle(1, "Game could not find the ball")
        # elif ball_in_map.transform.translation.x > 0:
        #     LATERAL_DEFENSE_LINE = -0.75
        # else:
        #     LATERAL_DEFENSE_LINE = 0.75

        if ball_in_map is not None and ball_in_map_past is not None:
            delta = to_se2(ball_in_map).translation() - to_se2(ball_in_map_past).translation()
            ball_speed = np.linalg.norm(delta) / BALL_LOOK_BEHIND
        else:
            ball_speed = np.nan

        # publish_goal_pose(tf2_buffer, ball_in_map)

        game_state_pub.publish(GameState(SCORES, ball_speed))

        rate.sleep()


if __name__ == "__main__":
    try:
        game()
    except rospy.ROSInterruptException:
        pass
