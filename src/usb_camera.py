#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

bridge = CvBridge()


def usb_camera() -> None:
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv.CAP_PROP_FPS, 20)
    rospy.init_node('usb_camera')
    pub = rospy.Publisher('/image', Image, queue_size=1)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            pub.publish(bridge.cv2_to_imgmsg(frame))
        rate.sleep()

    cap.release()


if __name__ == '__main__':
    try:
        usb_camera()
    except rospy.ROSInterruptException:
        pass
