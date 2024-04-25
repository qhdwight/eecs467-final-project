#!/usr/bin/env python3
import numpy as np

import cv2 as cv
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image


def usb_camera() -> None:
    rospy.init_node('usb_camera')

    cap = cv.VideoCapture('v4l2src device=/dev/video4 '
                          '! image/jpeg,width=1280,height=720,framerate=30/1 '
                          '! jpegdec '
                          '! video/x-raw,format=I420 '
                          '! appsink',
                          cv.CAP_GSTREAMER)

    topic = rospy.get_param('~topic', '/image')
    pub = rospy.Publisher(topic, Image, queue_size=1)

    bridge = CvBridge()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            dist = np.array([-0.52908235, 0.28727436, 0.00566978, -0.00345952, -0.02408352])
            mtx = np.array([
                [945.69568215, 0., 659.99381561],
                [0., 951.85384486, 365.3696245],
                [0., 0., 1.],
            ])
            dst = cv.undistort(frame, mtx, dist, None, mtx)
            pub.publish(bridge.cv2_to_imgmsg(dst))

        rate.sleep()

    cap.release()


if __name__ == '__main__':
    try:
        usb_camera()
    except rospy.ROSInterruptException:
        pass
