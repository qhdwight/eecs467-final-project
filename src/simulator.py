#!/usr/bin/env python3

from pathlib import Path

import pybullet as p
import pybullet_data as pd

import rospy
from hockey_cup.msg import WheelVelocities
from sensor_msgs.msg import Image

IMAGE_UPDATE_RATE = 20
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


def simulator() -> None:
    rospy.init_node('simulator')

    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(1)

    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")

    package_path = Path(__file__).parent.parent
    p.setAdditionalSearchPath(str(package_path / 'urdf'))
    mbot = p.loadURDF("mbot.urdf", [0, 0, 0.5])

    p.loadURDF("ball.urdf", [0.3, 0, 0.1])

    joint_name_to_id = {p.getJointInfo(mbot, i)[1].decode('utf-8'): i for i in range(p.getNumJoints(mbot))}

    # Movement

    def cmd_wheel_vels_callback(message: WheelVelocities) -> None:
        p.setJointMotorControl2(mbot, joint_name_to_id['base_to_left_wheel'],
                                p.VELOCITY_CONTROL, targetVelocity=-message.left)
        p.setJointMotorControl2(mbot, joint_name_to_id['base_to_right_wheel'],
                                p.VELOCITY_CONTROL, targetVelocity=message.right)

    rospy.Subscriber('cmd_wheel_vels', WheelVelocities, cmd_wheel_vels_callback)

    # Image

    image_pub = rospy.Publisher('image', Image, queue_size=1)

    def publish_image() -> None:
        data = p.getCameraImage(IMAGE_WIDTH, IMAGE_HEIGHT)
        width, height, rgba, *_ = data
        image_pub.publish(Image(
            header=rospy.Header(
                stamp=rospy.Time.now(),
            ),
            width=width,
            height=height,
            encoding='rgba8',
            step=width * 4,
            data=rgba.tobytes(),
        ))

    rospy.Timer(rospy.Duration(1 / IMAGE_UPDATE_RATE), lambda _: publish_image())

    rospy.spin()


if __name__ == '__main__':
    try:
        simulator()
    except rospy.ROSInterruptException:
        pass
