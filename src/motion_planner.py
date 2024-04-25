#!/usr/bin/env python3

from manifpy import SE2, SE2Tangent

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from transforms import *

import numpy as np


def motion_planner() -> None:
    rospy.init_node("motion_planner")

    tf2_buffer = Buffer()
    TransformListener(tf2_buffer)

    broadcaster = TransformBroadcaster()

    number = rospy.get_param('~number', 0)
    cmd_vel_topic = f'cmd_vel_{number}'
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    K = 256  # Number of trajectories to sample
    T = 8  # Size of trajectory horizon

    sigma = np.diag([0.4, 3.14])
    sigma_inv = np.linalg.inv(sigma)

    alpha = 1
    lambda_ = 20
    gamma = lambda_ * (1 - alpha)

    commands = None

    def terminal_cost(bot_in_map: SE2, goal_in_map: SE2) -> float:
        d = goal_in_map - bot_in_map
        coeffs = d.coeffs()
        coeffs *= [2, 2, 1]
        coeffs = np.clip(coeffs, [-1, -1, -np.pi], [1, 1, np.pi])
        return np.linalg.norm(coeffs, ord=2)
    
    def seed(bot_in_map: SE2, goal_in_map: SE2) -> NDArray:
        v, _, w = (goal_in_map - bot_in_map).coeffs()
        return np.array([v, w])

    def mppi(bot_in_map: SE2, goal_in_map: SE2) -> Twist:
        nonlocal commands

        if commands is None:
            commands = np.zeros((T, 2))
            x = bot_in_map
            for t in range(T):
                commands[t] = seed(x, goal_in_map)
                x = x + SE2Tangent(commands[t, 0], 0, commands[t, 1]) / 10

        # Sample noise to add to control input
        sample_perturbations = np.random.multivariate_normal(np.zeros(2), sigma, (K, T))
        # Commands with added noise
        sample_commands = np.zeros((K, T, 2))
        # Costs for each trajectory
        costs = np.zeros(K)

        for k in range(K):
            sample_final_pose = bot_in_map
            # Rollout trajectory "k" with given perturbations
            for t in range(T):
                sample_commands[k, t] = commands[t] + sample_perturbations[k, t]

                sample_command = SE2Tangent(sample_commands[k, t, 0], 0, sample_commands[k, t, 1])

                sample_final_pose = sample_final_pose + (sample_command / 10)

                # costs[k] += gamma * (commands[t].T @ sigma_inv @ sample_commands[k, t])

            costs[k] += terminal_cost(sample_final_pose, goal_in_map)

        rho = costs.min()

        soft_maxed = np.exp((-1 / lambda_) * (costs - rho))

        w = soft_maxed / np.sum(soft_maxed)

        w_epsilon = np.average(sample_perturbations  , axis=0, weights=w)

        commands += w_epsilon

        x = bot_in_map
        for t in range(T):
            x = x + SE2Tangent(commands[t, 0], 0, commands[t, 1]) / 10
            if t % 2 == 0:
                broadcaster.sendTransform(to_tf(x, "map", f"bot_{number}_mppi_{t}"))

        v, w = commands[0]

        v = np.clip(v, -0.6, 0.6)
        w = np.clip(w, -1.5, 1.5)

        print(v, w)

        commands = np.roll(commands, -1, axis=0)
        commands[-1] = seed(bot_in_map, goal_in_map)

        # return Twist(
        #     linear=Vector3(x=sample_commands[np.argmin(costs), 0, 0]),
        #     angular=Vector3(z=sample_commands[np.argmin(costs), 0, 1])
        # )

        return Twist(
            linear=Vector3(x=v),
            angular=Vector3(z=w)
        )

    # def control_law(bot_in_map: SE2, goal_in_map: SE2) -> Twist:
    #     tangent = goal_in_map - bot_in_map
    #     tangent = tangent.coeffs()
    #     tangent *= [2, 12, 5]
    #     tangent[2] += copysign(tangent[1], tangent[0])
    #     tangent[1] = 0
    #     tangent = np.clip(tangent, [-0.6, -0.6, -np.pi], [0.6, 0.6, np.pi])
    #     v, _, w = tangent
    #     return Twist(
    #         linear=Vector3(x=v),
    #         angular=Vector3(z=w)
    #     )

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            goal_in_map = to_se2(tf2_buffer.lookup_transform("map", f"goal_{number}", rospy.Time(0)))
            bot_in_map = to_se2(tf2_buffer.lookup_transform("map", f"bot_{number}", rospy.Time(0)))

            twist = mppi(bot_in_map, goal_in_map)
            cmd_vel_pub.publish(twist)

        except Exception as e:
            rospy.logwarn_throttle(1, f"Motion planner error: {e}")

        rate.sleep()


if __name__ == "__main__":
    try:
        motion_planner()
    except rospy.ROSInterruptException:
        pass
