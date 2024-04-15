#!/usr/bin/env python3

import cv2 as cv

import geometry_msgs.msg
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
import numpy as np

from geometry_msgs.msg import TransformStamped


def bot_detector() -> None:
    rospy.init_node('bot_detector')

    broadcaster = TransformBroadcaster()
    bridge = CvBridge()
    detector = cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50))

    debug_image_topic = rospy.get_param('~debug_image_topic', '/debug_image')
    debug_image_publisher = rospy.Publisher(debug_image_topic, Image, queue_size=1)

    def image_callback(message: Image) -> None:
        frame = bridge.imgmsg_to_cv2(message)
        if message.encoding == 'rgba8':
            frame = cv.cvtColor(frame, cv.COLOR_RGBA2BGR)
        corners, ids, _rejected = detector.detectMarkers(frame)

        if corners:
            for corner, id in zip(corners, ids):
                side_length = 0.1
                square = np.array([
                    [-side_length / 2, -side_length / 2, 0],
                    [side_length / 2, -side_length / 2, 0],
                    [side_length / 2, side_length / 2, 0],
                    [-side_length / 2, side_length / 2, 0],
                ])
                fx = 1.0
                fy = 1.0
                cx = frame.shape[1] / 2
                cy = frame.shape[0] / 2
                camera_matrix = np.array([
                    [fx, 0, cx],
                    [0, fy, cy],
                    [0, 0, 1],
                ])
                distortions = np.zeros((4, 1))
                _, rvec, tvec = cv.solvePnP(
                    square,
                    corner,
                    camera_matrix,
                    distortions,
                )
                if np.isnan(tvec).any() or np.isnan(rvec).any():
                    continue

                angle = np.linalg.norm(rvec)
                axis = rvec / angle

                axis *= -1
                angle += np.pi

                tvec[0] *= -1

                position = np.vstack((tvec[:2], 0))
                orientation = np.vstack((np.sin(angle / 2) * axis, np.cos(angle / 2)))

                broadcaster.sendTransform(TransformStamped(
                    header=rospy.Header(
                        stamp=rospy.Time.now(),
                        frame_id='map',
                    ),
                    child_frame_id=f'bot_{id[0]}',
                    transform=geometry_msgs.msg.Transform(
                        translation=geometry_msgs.msg.Vector3(*position.ravel()),
                        rotation=geometry_msgs.msg.Quaternion(*orientation.ravel()),
                    ),
                ))
            debug_image = cv.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        else:
            debug_image = frame
        debug_image_publisher.publish(bridge.cv2_to_imgmsg(debug_image))

    image_topic = rospy.get_param('~image_topic', '/image')
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        bot_detector()
    except rospy.ROSInterruptException:
        pass
