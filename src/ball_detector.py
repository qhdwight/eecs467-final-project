#!/usr/bin/env python3

from math import tan

import cv2 as cv
import geometry_msgs.msg
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster

broadcaster = TransformBroadcaster()
bridge = CvBridge()
img_pub = rospy.Publisher("/thresh_img", Image, queue_size=1)

MIN_THRESH = (100, 130, 40)
MAX_THRESH = (190, 255, 200)
CAMERA_FX = 569.21735
CAMERA_FY = 569.15094
CAMERA_CX = 639.81628
CAMERA_CY = 352.37375

CAMERA_HEIGHT = 1.4


def image_callback(data: Image) -> None:
    # convert to opencv using cv_bridge
    cv_image = bridge.imgmsg_to_cv2(data, "bgra8")

    # Apply GaussianBlur to reduce noise
    blur = cv.GaussianBlur(cv_image, (11, 11), 0)
    # convert to hsv
    # This is a BGR image, but we are purposefully using RGB2HSV to make red a consistent channel that has no wraparound.
    # Effectively red is being represented by blue in HSV which is around 170 on hue scale
    hsv = cv.cvtColor(blur, cv.COLOR_RGB2HSV)

    # apply thresholding
    thresh = cv.inRange(hsv, MIN_THRESH, MAX_THRESH)
    contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    best_cnt = None
    if len(contours) > 0:
        # find contour of ball
        for c in contours:
            if 900 > cv.contourArea(c) > 300:
                best_cnt = c
                break
        if best_cnt is None:
            rospy.logwarn_throttle(1, "No valid contour found")
            img_pub.publish(bridge.cv2_to_imgmsg(thresh))
            return
        cv.drawContours(cv_image, best_cnt, -1, (0, 255, 0), 2)

        # Compute center of contour
        M = cv.moments(best_cnt)
        pX = int(M["m10"] / M["m00"])
        pY = int(M["m01"] / M["m00"])
        cv.circle(cv_image, (pX, pY), 3, (255, 255, 255), -1)

        bearing_x = (pX - CAMERA_CX) / CAMERA_FX
        pos_x = tan(bearing_x) * CAMERA_HEIGHT
        bearing_y = -(pY - CAMERA_CY) / CAMERA_FY
        pos_y = tan(bearing_y) * CAMERA_HEIGHT

        broadcaster.sendTransform(TransformStamped(
            header=rospy.Header(
                stamp=rospy.Time.now(),
                frame_id='map',
            ),
            child_frame_id=f'ball',
            transform=geometry_msgs.msg.Transform(
                translation=geometry_msgs.msg.Vector3(pos_x, pos_y, 0),
                rotation=geometry_msgs.msg.Quaternion(0, 0, 0, 1),
            ),
        ))
    else:
        rospy.logwarn_throttle(1, "No contours found")
    # publish img with drawn points
    img_msg = bridge.cv2_to_imgmsg(thresh)
    img_pub.publish(img_msg)


def ball_detector() -> None:
    rospy.init_node('ball_detector')
    rospy.Subscriber('/image', Image, image_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        ball_detector()
    except rospy.ROSInterruptException:
        pass
