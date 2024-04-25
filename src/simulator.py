#!/usr/bin/env python3

from pathlib import Path

import pybullet as p
import pybullet_data as pd
import geometry_msgs.msg

import rospy
from hockey_cup.msg import WheelVelocities
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist

import numpy as np

IMAGE_UPDATE_RATE = 10
TF_UPDATE_RATE = 10
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Overhead camera
# Position in the sky and look towards the origin
CAMERA_POSE = [0, 0, 1.2]
CAMERA_VIEW_MAT = p.computeViewMatrix(CAMERA_POSE, [0, 0, 0], [0, 1, 0])
CAMERA_PROJ_MAT = p.computeProjectionMatrixFOV(100, IMAGE_WIDTH / IMAGE_HEIGHT, 0.1, 100)

tf_broadcaster = TransformBroadcaster()

class MBot:
    def cmd_wheel_vels_callback(self, message: WheelVelocities) -> None:
        p.setJointMotorControl2(self.urdf, self.joint_name_to_id['base_to_left_wheel'],
                                p.VELOCITY_CONTROL, targetVelocity=-message.left)
        p.setJointMotorControl2(self.urdf, self.joint_name_to_id['base_to_right_wheel'],
                                p.VELOCITY_CONTROL, targetVelocity=message.right)
        
    def __init__(self, number, position, orientation):
        self.number = number
        self.urdf = p.loadURDF(f"mbot_{self.number}.urdf", position, orientation)
        self.joint_name_to_id = {p.getJointInfo(self.urdf, i)[1].decode('utf-8'): i for i in range(p.getNumJoints(self.urdf))}

        rospy.Subscriber(f'cmd_wheel_vels_{self.number}', WheelVelocities, lambda m: self.cmd_wheel_vels_callback(m))

# class Ball:
#     def cmd_ball_vel_callback(self, message: Twist) -> None:
#         p.setJointMotorControl2(self.urdf, self.joint_name_to_id['base_to_ball'], 
#                                 p.VELOCITY_CONTROL, targetVelocity=message.linear.x)

#     def __init__(self, position, orientation):
#         self.urdf = p.loadURDF("ball.urdf", position, orientation)
#         rospy.Subscriber('cmd_ball_vel', Twist, self.cmd_ball_vel_callback())

def simulator() -> None:
    rospy.init_node('simulator')

    use_ground_truth = rospy.get_param('~ground_truth', False)

    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(1)

    p.setAdditionalSearchPath(pd.getDataPath())
    p.loadURDF("plane.urdf")

    package_path = Path(__file__).parent.parent
    p.setAdditionalSearchPath(str(package_path / 'urdf'))

    mbots = [
        MBot(0, [1, 0.1, 0.5], p.getQuaternionFromEuler([0, 0, np.pi])),
        MBot(1, [-1, -0.1, 0.5], p.getQuaternionFromEuler([0, 0, 0])),
    ]

    ball = p.loadURDF("ball.urdf", [0.3, 0.3, 0.1])

    p.loadURDF("field.urdf", [0, 0, 0])

    # TF

    def publish_tf() -> None:
        tf_broadcaster.sendTransform(TransformStamped(
            header=rospy.Header(
                stamp=rospy.Time.now(),
                frame_id='map',
            ),
            child_frame_id=f'camera',
            transform=geometry_msgs.msg.Transform(
                translation=geometry_msgs.msg.Vector3(*CAMERA_POSE),
                rotation=geometry_msgs.msg.Quaternion(0, 0, 0, 1),
            ),
        ))
        def publish_urdf_to_tf(urdf, name: str) -> None:
            pos, rot = p.getBasePositionAndOrientation(urdf)
            tf_broadcaster.sendTransform(TransformStamped(
                header=rospy.Header(
                    stamp=rospy.Time.now(),
                    frame_id='map',
                ),
                child_frame_id=name,
                transform=geometry_msgs.msg.Transform(
                    translation=geometry_msgs.msg.Vector3(*pos),
                    rotation=geometry_msgs.msg.Quaternion(*rot),
                ),
            ))
        for mbot in mbots:
            publish_urdf_to_tf(mbot.urdf, f'bot_{mbot.number}{"" if use_ground_truth else "_ground"}')
        publish_urdf_to_tf(ball, 'ball' if use_ground_truth else 'ball_ground')


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

    if not use_ground_truth:
        rospy.Timer(rospy.Duration(1 / IMAGE_UPDATE_RATE), lambda _: publish_image())
    rospy.Timer(rospy.Duration(1 / TF_UPDATE_RATE), lambda _: publish_tf())

    rospy.spin()


if __name__ == '__main__':
    try:
        simulator()
    except rospy.ROSInterruptException:
        pass
