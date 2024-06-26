#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from transforms import *

FIELD_LENGTH = 2.44
FIELD_WIDTH = 1.22
BUFFER = 0.1


def motion_planner() -> None:
    rospy.init_node("motion_planner")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    tf2_broadcast = TransformBroadcaster()

    number = rospy.get_param('~number', 0)
    cmd_vel_topic = f'cmd_vel_{number}'
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)


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

        MAX_ANGULAR = 1.2
        MAX_LINEAR = 0.3

        vx, vy, w = (goal_in_map - bot_in_map).coeffs()
        if abs(vy) < 0.1:
            return Twist(
                linear=Vector3(x=np.clip(vx * 2, -MAX_LINEAR, MAX_LINEAR)),
                angular=Vector3(z=np.clip(w * 2, -MAX_ANGULAR, MAX_ANGULAR)),
            )

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
            vx, vy, w = (to_goal_pose - bot_in_map).coeffs()
            if abs(w) > 0.4:
                # Rotate towards the goal
                return Twist(
                    angular=Vector3(z=np.clip(w * 4, -MAX_ANGULAR, MAX_ANGULAR))
                )
            else:
                # Straight line with minor angular correction
                return Twist(
                    linear=Vector3(x=np.clip(vx * 2, -MAX_LINEAR, MAX_LINEAR)),
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

            goal_in_map.coeffs()[0] = np.clip(goal_in_map.coeffs()[0],
                                              -FIELD_LENGTH / 2 + BUFFER,
                                              FIELD_LENGTH / 2 - BUFFER)
            goal_in_map.coeffs()[1] = np.clip(goal_in_map.coeffs()[1],
                                              -FIELD_WIDTH / 2 + BUFFER,
                                              FIELD_WIDTH / 2 - BUFFER)

            cmd_vel = rtr(bot_in_map, goal_in_map)

            cmd_vel_pub.publish(cmd_vel)

            # Stuck detection

            SPEED_LOOKBACK_DURATION = 0.3
            bot_in_map_prev = to_se2(tf2_buffer.lookup_transform(
                "map", f"bot_{number}",
                rospy.Time.now() - rospy.Duration(SPEED_LOOKBACK_DURATION))
            )
            bot_speed = np.linalg.norm((bot_in_map - bot_in_map_prev).coeffs(), ord=2) / SPEED_LOOKBACK_DURATION

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
