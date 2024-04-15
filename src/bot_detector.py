#!/usr/bin/env python3

import cv2 as cv
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

    debug_image_publisher = rospy.Publisher('/debug_image', Image, queue_size=1)

    def image_callback(message: Image) -> None:
        frame = bridge.imgmsg_to_cv2(message)
        corners, ids, _rejected = detector.detectMarkers(frame)

        if corners:
            for corner, id in zip(corners, ids):
                side_length = 1
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
                angle = np.linalg.norm(rvec)
                axis = rvec / angle

                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = 'map'
                transform.child_frame_id = f'bot_{id[0]}'
                transform.transform.translation.x = tvec[0][0]
                transform.transform.translation.y = tvec[1][0]
                transform.transform.translation.z = 0
                transform.transform.rotation.x = axis[0] * np.sin(angle / 2)
                transform.transform.rotation.y = axis[1] * np.sin(angle / 2)
                transform.transform.rotation.z = axis[2] * np.sin(angle / 2)
                transform.transform.rotation.w = np.cos(angle / 2)
                broadcaster.sendTransform(transform)
            debug_image = cv.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        else:
            debug_image = frame
        debug_image_publisher.publish(bridge.cv2_to_imgmsg(debug_image))

    rospy.Subscriber('/image', Image, image_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        bot_detector()
    except rospy.ROSInterruptException:
        pass
