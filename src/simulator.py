#!/usr/bin/env python3

from pathlib import Path

import pybullet as p
import pybullet_data as pd

import rospy
from hockey_cup.msg import WheelVelocities


def simulator() -> None:
    rospy.init_node('simulator')

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(1)
    p.loadURDF("plane.urdf")
    package_path = Path(__file__).parent.parent
    mbot = p.loadURDF(str(package_path / "urdf" / "mbot.urdf"), [0, 0, 0.5])

    joint_name_to_id = {p.getJointInfo(mbot, i)[1].decode('utf-8'): i for i in range(p.getNumJoints(mbot))}

    def cmd_wheel_vels_callback(message: WheelVelocities) -> None:
        p.setJointMotorControl2(mbot, joint_name_to_id['base_to_left_wheel'],
                                p.VELOCITY_CONTROL, targetVelocity=-message.left)
        p.setJointMotorControl2(mbot, joint_name_to_id['base_to_right_wheel'],
                                p.VELOCITY_CONTROL, targetVelocity=message.right)

    rospy.Subscriber('cmd_wheel_vels', WheelVelocities, cmd_wheel_vels_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        simulator()
    except rospy.ROSInterruptException:
        pass
