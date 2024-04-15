#!/usr/bin/env python3

import cv2 as cv
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image


def usb_camera() -> None:
    rospy.init_node('usb_camera')

    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv.CAP_PROP_FPS, 30)

    topic = rospy.get_param('~topic', '/image')
    pub = rospy.Publisher(topic, Image, queue_size=1)

    bridge = CvBridge()

    rate = rospy.Rate(30)

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
