#!/usr/bin/env python3

import pybullet as p
import pybullet_data as pd

import rospy

from pathlib import Path


def simulator() -> None:
    rospy.init_node('simulator')

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(1)
    p.loadURDF("plane.urdf")
    package_path = Path(__file__).parent.parent
    p.loadURDF(str(package_path / "urdf" / "mbot.urdf"), [0, 0, 0.5])

    def

    rospy.spin()


if __name__ == '__main__':
    try:
        simulator()
    except rospy.ROSInterruptException:
        pass
