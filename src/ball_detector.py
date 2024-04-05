#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge

from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image

broadcaster = TransformBroadcaster()
bridge = CvBridge()
pub = rospy.Publisher("/thresh_img", Image, queue_size=1)
MIN_THRESH = (120,120,20)
MAX_THRESH = (150,255,255)

def image_callback(data: Image) -> None:
    # TODO:
    # 1. convert to opencv using cv_bridge
    cv_image = bridge.imgmsg_to_cv2(data, "8UC3")

    # Apply GaussianBlur to reduce noise
    blur = cv.GaussianBlur(cv_image, (11, 11), 0)
    # 2. convert to hsv
    # This is a BGR image, but we are purposefully using RGB2HSV to make red a consistent channel that has no wraparound.
    # Effectively red is being represented by blue in HSV which is around 170 on hue scale
    hsv = cv.cvtColor(blur, cv.COLOR_RGB2HSV)

    # 3. apply thresholding
    thresh = cv.inRange(hsv, MIN_THRESH, MAX_THRESH)
    contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        max_cnt = max(contours, key=cv.contourArea)
        if cv.contourArea(max_cnt) < 200:
            print("Contour area too small")
            return
        # 4. find contours
        cv.drawContours(cv_image, max_cnt, -1, (0, 255, 0), 2)
        img_msg = bridge.cv2_to_imgmsg(cv_image)
        pub.publish(img_msg)
    else:
        print("No contours found")
    # 5. find circle
    # 6. convert to map coordinates
    # 7. publish the circle as a transform "ball" in "map" using "broadcaster"


def ball_detector() -> None:
    rospy.init_node('ball_detector')
    rospy.Subscriber('/image', Image, image_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        ball_detector()
    except rospy.ROSInterruptException:
        pass
