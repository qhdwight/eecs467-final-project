#!/usr/bin/env python3

from pathlib import Path

import pybullet as p
import pybullet_data as pd
import geometry_msgs.msg

import rospy
from hockey_cup.msg import WheelVelocities
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

IMAGE_UPDATE_RATE = 10
TF_UPDATE_RATE = 10
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Overhead camera
# Position in the sky and look towards the origin
CAMERA_POSE = [0, 0, 1.2]
CAMERA_VIEW_MAT = p.computeViewMatrix(CAMERA_POSE, [0, 0, 0], [1, 0, 0])
CAMERA_PROJ_MAT = p.computeProjectionMatrixFOV(100, IMAGE_WIDTH / IMAGE_HEIGHT, 0.1, 100)

tf_broadcaster = TransformBroadcaster()

def simulator() -> None:
    rospy.init_node('simulator')

    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(1)

    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")

    package_path = Path(__file__).parent.parent
    p.setAdditionalSearchPath(str(package_path / 'urdf'))
    mbot_1 = p.loadURDF("mbot_1.urdf", [-1, 0, 0.5])
    mbot_2 = p.loadURDF("mbot_2.urdf", [1, 0, 0.5], p.getQuaternionFromEuler([0, 0, 3.14]))

    p.loadURDF("ball.urdf", [0.3, 0, 0.1])

    joint_name_to_id = {p.getJointInfo(mbot_1, i)[1].decode('utf-8'): i for i in range(p.getNumJoints(mbot_1))}

    # Movement

    def cmd_wheel_vels_callback(message: WheelVelocities) -> None:
        p.setJointMotorControl2(mbot_1, joint_name_to_id['base_to_left_wheel'],
                                p.VELOCITY_CONTROL, targetVelocity=-message.left)
        p.setJointMotorControl2(mbot_1, joint_name_to_id['base_to_right_wheel'],
                                p.VELOCITY_CONTROL, targetVelocity=message.right)

    rospy.Subscriber('cmd_wheel_vels', WheelVelocities, cmd_wheel_vels_callback)

    # TF

    def publish_tf() -> None:
        tf_broadcaster.sendTransform(TransformStamped(
            header=rospy.Header(
                stamp=rospy.Time.now(),
                frame_id='map',
            ),
            child_frame_id=f'camera_frame',
            transform=geometry_msgs.msg.Transform(
                translation=geometry_msgs.msg.Vector3(*CAMERA_POSE),
                rotation=geometry_msgs.msg.Quaternion(*[0, 0, 0, 1]),
            ),
        ))

    # Image

    image_pub = rospy.Publisher('image', Image, queue_size=1)

    def publish_image() -> None:
        data = p.getCameraImage(IMAGE_WIDTH, IMAGE_HEIGHT, CAMERA_VIEW_MAT, CAMERA_PROJ_MAT)
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
    rospy.Timer(rospy.Duration(1 / TF_UPDATE_RATE), lambda _: publish_tf())

    rospy.spin()


if __name__ == '__main__':
    try:
        simulator()
    except rospy.ROSInterruptException:
        pass
