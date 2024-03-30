import rospy
import cv2 as cv

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image


def image_callback(data: Image) -> None:
    pass


def ball_detector() -> None:
    rospy.init_node('ball_detector')
    rospy.Subscriber('/image', Image, image_callback)
    rospy.Publisher('/ball_pose', Pose2D)
    rospy.spin()


if __name__ == '__main__':
    try:
        ball_detector()
    except rospy.ROSInterruptException:
        pass
