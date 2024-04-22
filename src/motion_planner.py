#!/usr/bin/env python3

import numpy as np
from roboticstoolbox import ReedsSheppPlanner


import time
from geometry_msgs.msg import Twist
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
        tangent = goal_in_map - bot_in_map
        v, _, w = tangent.coeffs()
        v *= 1.5
        w *= 4
        return Twist(
            linear=Vector3(x=v),
            angular=Vector3(z=w)
        )
        # angle_to_goal = angle_to(bot_in_map, goal_in_map)
        # goal_is_in_front = np.abs(angle_to_goal) < np.pi / 2

        # if goal_is_in_front:
        #     tangent = goal_in_map - bot_in_map
        #     dx, _, dt = tangent.coeffs() * P
        #     return Twist(
        #         linear=Vector3(x=dx),
        #         angular=Vector3(z=dt)
        #     )
        # else:
        #     rotated_bot_in_map = bot_in_map * SE2(0, 0, np.pi)
        #     tangent = goal_in_map - rotated_bot_in_map
        #     print(rotated_bot_in_map.angle(), bot_in_map.angle())
        #     dx, _, dt = tangent.coeffs() * P
        #     return Twist(
        #         linear=Vector3(x=-dx),
        #         angular=Vector3(z=-dt)
        #     )

        # Motion planning is based off section 13.3.4 in "Modern Robotics"
        #
        # K1 = 0.3
        # K2 = 1.4
        # K3 = 1.8
        #
        # # Equation 13.30
        # b, g = se2_to_np(bot_in_map), se2_to_np(goal_in_map)
        # xe, ye, pe = np.array([
        #     [np.cos(g[2]), np.sin(g[2]), 0],
        #     [-np.sin(g[2]), np.cos(g[2]), 0],
        #     [0, 0, 1],
        # ]) @ (b - g)
        #
        # vd = 0.5
        # wd = 0
        #
        # # Equation 13.31
        # v = (vd - K1 * abs(vd) * (xe + ye * np.tan(pe))) / np.cos(pe)
        # w = wd - (K2 * vd * ye + K3 * abs(vd) * np.tan(pe)) * np.cos(pe) ** 2
        #
        # v = np.clip(v, -5, 5)
        # w = np.clip(w, -15, 15)
        #
        # # v *= -1
        # w *= -1
        #
        # return Twist(
        #     linear=Vector3(x=v),
        #     angular=Vector3(z=w),
        # )

    prev_goal_in_map = None
    trajectory = None

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            goal_in_map = to_se2(tf2_buffer.lookup_transform("map", f"goal_{number}", rospy.Time(0)))
            bot_in_map = to_se2(tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0)))

            delta = np.linalg.norm((goal_in_map - prev_goal_in_map).coeffs()) if prev_goal_in_map else np.inf
            if delta > 0.1:
                trajectory = (*planner.query(se2_to_np(bot_in_map), se2_to_np(goal_in_map)), time.time())

            # twist = control_law(bot_in_map, goal_in_map)
            # cmd_vel_pub.publish(twist)

            # path, status = planner.query(se2_to_np(bot_in_map), se2_to_np(goal_in_map))

            path, status, start_time = trajectory

            for i, link in enumerate(path):
                tf2_broadcast.sendTransform(to_tf(SE2(*link), "map", f"setpoint_{number}_{i}"))

            target_index = min(int((time.time() - start_time) / 0.3), len(path) - 1)
            print(target_index, len(path))

            target_pose = SE2(*path[target_index])
            cmd_vel = control_law(bot_in_map, target_pose)
            cmd_vel_pub.publish(cmd_vel)

            # LOOK_AHEAD = 5

            # if path.shape[0] > LOOK_AHEAD:
            #     front = SE2(*path[LOOK_AHEAD])
            #     cmd_vel = control_law(bot_in_map, front)
            #     cmd_vel_pub.publish(cmd_vel)

            #     tf2_broadcast.sendTransform(to_tf(front, "map", "setpoint"))
            # else:
            #     cmd_vel_pub.publish(Twist())

            prev_goal_in_map = goal_in_map

        except Exception as e:
            rospy.logwarn_throttle(1, f"Motion planner error: {e}")

        rate.sleep()


if __name__ == "__main__":
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
