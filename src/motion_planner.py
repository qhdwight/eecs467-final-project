#!/usr/bin/env python3

import time

from geometry_msgs.msg import Twist
from roboticstoolbox import ReedsSheppPlanner
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from transforms import *


def motion_planner() -> None:
    rospy.init_node("motion_planner")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    tf2_broadcast = TransformBroadcaster()

    number = rospy.get_param('~number', 0)
    cmd_vel_topic = f'cmd_vel_{number}'
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    planner = ReedsSheppPlanner(curvature=3.0)

    def control_law(bot_in_map: SE2, goal_in_map: SE2) -> Twist:
        twist = goal_in_map - bot_in_map
        v, _, w = twist.coeffs() * [1.5, 0, 1.5]
        v = np.clip(v, -0.2, 0.2)
        w = np.clip(w, -1.0, 1.0)
        return Twist(
            linear=Vector3(x=v),
            angular=Vector3(z=w)
        )

    def rtr(bot_in_map: SE2, goal_in_map: SE2) -> Twist:
        bot_to_goal = goal_in_map.translation() - bot_in_map.translation()

        MAX_ANGULAR = 0.6
        MAX_LINEAR = 0.2

        v, _, w = (goal_in_map - bot_in_map).coeffs()
        dp = np.linalg.norm(bot_to_goal)
        if dp < 0.07:
            # Final turn to align with the goal orientation
            if abs(w) > 0.1:
                return Twist(
                    angular=Vector3(z=np.clip(w * 4, -MAX_ANGULAR, MAX_ANGULAR))
                )
            else:
                return Twist()
        else:
            to_goal_pose = SE2(goal_in_map.x(), goal_in_map.y(), np.arctan2(bot_to_goal[1], bot_to_goal[0]))
            v, vy, w = (to_goal_pose - bot_in_map).coeffs()
            if abs(w) > 0.3:
                # Rotate towards the goal
                return Twist(
                    angular=Vector3(z=np.clip(w * 4, -MAX_ANGULAR, MAX_ANGULAR))
                )
            else:
                # Straight line with minor angular correction
                return Twist(
                    linear=Vector3(x=np.clip(v * 2, -MAX_LINEAR, MAX_LINEAR)),
                    angular=Vector3(z=np.clip(w * 2, -MAX_ANGULAR, MAX_ANGULAR)),
                )

    prev_goal_in_map = None
    trajectory = None

    stuck_count = 0

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            goal_in_map = to_se2(tf2_buffer.lookup_transform("map", f"goal_{number}", rospy.Time(0)))
            bot_in_map = to_se2(tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0)))

            SPEED_LOOKBACK_DURATION = 0.3
            bot_in_map_prev = to_se2(tf2_buffer.lookup_transform(
                "map", f"bot_{number}",
                rospy.Time.now() - rospy.Duration(SPEED_LOOKBACK_DURATION))
            )
            bot_speed = np.linalg.norm((bot_in_map - bot_in_map_prev).coeffs(), ord=2) / SPEED_LOOKBACK_DURATION

            # if se2_distance(bot_in_map, goal_in_map) < 0.1:
            #     rospy.loginfo_throttle(1, "Reached target")
            #     cmd_vel = Twist()
            # else:
            # if prev_goal_in_map is None or se2_distance(prev_goal_in_map, goal_in_map) > 0.1:
            #     rospy.loginfo(f"New goal: {se2_to_np(goal_in_map)}")
            #     path, status = planner.query(se2_to_np(bot_in_map), se2_to_np(goal_in_map))
            #
            #     prev_goal_in_map = goal_in_map
            #
            # for i, link in enumerate(path):
            #     tf2_broadcast.sendTransform(to_tf(SE2(*link), "map", f"setpoint_{number}_{i}"))
            #
            # segment_distances = [
            #     np.linalg.norm((SE2(*path[i]) - bot_in_map).coeffs(), ord=2)
            #     for i in range(len(path))
            # ]
            # look_ahead = 2
            # target_index = min(np.argmin(segment_distances) + look_ahead, len(path) - 1)
            # target_pose = SE2(*path[target_index])
            #
            # cmd_vel = control_law(bot_in_map, target_pose)
            cmd_vel = rtr(bot_in_map, goal_in_map)

            cmd_vel_pub.publish(cmd_vel)

            # Stuck detection

            if bot_speed < 0.05 < np.sqrt(cmd_vel.linear.x ** 2 + cmd_vel.angular.z ** 2):
                stuck_count += 1
                if stuck_count > 20:
                    rospy.logwarn("Stuck, attempting recovery")
                    rate = rospy.Rate(10)
                    now = rospy.Time.now()
                    while rospy.Time.now() - now < rospy.Duration(2):
                        cmd_vel_pub.publish(Twist(linear=Vector3(x=-0.1)))
                        rate.sleep()
            else:
                stuck_count = 0

        except Exception as e:
            rospy.logwarn_throttle(1, f"Motion planner error: {e}")

        rate.sleep()


if __name__ == "__main__":
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
