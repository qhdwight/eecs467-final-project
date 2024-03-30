#!/usr/bin/env python3

import rospy
import cv2 as cv

from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image

broadcaster = TransformBroadcaster()


def image_callback(data: Image) -> None:
    # TODO:
    # 1. convert to opencv using cv_bridge
    # 2. convert to hsv
    # 3. apply thresholding
    # 4. find contours
    # 5. find circle
    # 6. convert to map coordinates
    # 7. publish the circle as a transform "ball" in "map" using "broadcaster"
    pass


def ball_detector() -> None:
    rospy.init_node('ball_detector')
    rospy.Subscriber('/image', Image, image_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        ball_detector()
    except rospy.ROSInterruptException:
        pass
