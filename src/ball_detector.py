#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
import geometry_msgs.msg
from math import tan, pi

from tf2_ros import TransformBroadcaster, TransformListener
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image

broadcaster = TransformBroadcaster()
bridge = CvBridge()
img_pub = rospy.Publisher("/thresh_img", Image, queue_size=1)
MIN_THRESH = (100,110,20)
MAX_THRESH = (180,255,255)
CAMERA_FOV = 100
CAMERA_HEIGHT = 1.2

def image_callback(data: Image) -> None:
    # TODO:
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
    if len(contours) > 0:
        # find contours
        max_cnt = max(contours, key=cv.contourArea)
        if cv.contourArea(max_cnt) < 200:
            print("Contour area too small")
            return
        cv.drawContours(cv_image, max_cnt, -1, (0, 255, 0), 2)

        # Compute center of contour
        M = cv.moments(max_cnt)
        pX = int(M["m10"] / M["m00"])
        pY = int(M["m01"] / M["m00"])
        cv.circle(cv_image, (pX, pY), 3, (255, 255, 255), -1)

        bearing_x = (pX - cv_image.shape[1] / 2) / cv_image.shape[1] * CAMERA_FOV
        pos_x = tan(bearing_x * pi/ 180.0) * CAMERA_HEIGHT
        bearing_y = (cv_image.shape[0] / 2 - pY) / cv_image.shape[0] * CAMERA_FOV
        pos_y = tan(bearing_y * pi/ 180.0) * CAMERA_HEIGHT


        broadcaster.sendTransform(TransformStamped(
            header=rospy.Header(
                stamp=rospy.Time.now(),
                frame_id='map',
            ),
            child_frame_id=f'ball_frame',
            transform=geometry_msgs.msg.Transform(
                translation=geometry_msgs.msg.Vector3(pos_x, pos_y, 0),
                rotation=geometry_msgs.msg.Quaternion(*[0,0,0,1]),
            ),
        ))
    else:
        print("No contours found")
    # 6. convert to map coordinates
    # 7. publish the circle as a transform "ball" in "map" using "broadcaster"
            # publish img with drawn points
    img_msg = bridge.cv2_to_imgmsg(cv_image)
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
