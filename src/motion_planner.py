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
        v, _, w = twist.coeffs() * [1.5, 0, 4]
        return Twist(
            linear=Vector3(x=v),
            angular=Vector3(z=w)
        )

    prev_goal_in_map = None
    trajectory = None

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            goal_in_map = to_se2(tf2_buffer.lookup_transform("map", f"goal_{number}", rospy.Time(0)))
            bot_in_map = to_se2(tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0)))

            if prev_goal_in_map is None or not np.allclose(goal_in_map.coeffs(), prev_goal_in_map.coeffs(), atol=0.1):
                rospy.loginfo(f"New goal: {se2_to_np(goal_in_map)}")
                trajectory = (*planner.query(se2_to_np(bot_in_map), se2_to_np(goal_in_map)), time.time())

            # twist = control_law(bot_in_map, goal_in_map)
            # cmd_vel_pub.publish(twist)

            # path, status = planner.query(se2_to_np(bot_in_map), se2_to_np(goal_in_map))

            path, status, start_time = trajectory

            for i, link in enumerate(path):
                tf2_broadcast.sendTransform(to_tf(SE2(*link), "map", f"setpoint_{number}_{i}"))

            # target_index = min(int((time.time() - start_time) / 0.3), len(path) - 1)
            # print(target_index, len(path))

            segment_distances = [
                np.linalg.norm((SE2(*path[i]) - bot_in_map).coeffs(), ord=2)
                for i in range(len(path))
            ]
            look_ahead = 2
            target_index = min(np.argmin(segment_distances) + look_ahead, len(path) - 1)
            target_pose = SE2(*path[target_index])

            cmd_vel = control_law(bot_in_map, target_pose)
            cmd_vel_pub.publish(cmd_vel)

            prev_goal_in_map = goal_in_map

        except Exception as e:
            rospy.logwarn_throttle(1, f"Motion planner error: {e}")

        rate.sleep()


if __name__ == "__main__":
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
