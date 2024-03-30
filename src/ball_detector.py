#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge

from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image

broadcaster = TransformBroadcaster()
bridge = CvBridge()
pub = rospy.Publisher("/thresh_img", Image, queue_size=1)
MIN_THRESH = (5,50,50)
MAX_THRESH = (15,255,255)

def image_callback(data: Image) -> None:
    # TODO:
    # 1. convert to opencv using cv_bridge
    cv_image = bridge.imgmsg_to_cv2(data, "8UC3")
    # 2. convert to hsv
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    # 3. apply thresholding
    thresh = cv.inRange(hsv, MIN_THRESH, MAX_THRESH)
    # contour_img, contours = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # 4. find contours
    # cv.drawContours(thresh, contours, -1, (0, 255, 0), 3)

    # 5. find circle
    # 6. convert to map coordinates
    # 7. publish the circle as a transform "ball" in "map" using "broadcaster"
    img_msg = bridge.cv2_to_imgmsg(thresh)
    pub.publish(img_msg)
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
