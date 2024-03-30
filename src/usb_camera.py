import rospy
import cv2 as cv

from sensor_msgs.msg import Image


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
            msg = Image()
            msg.data = frame.tobytes()
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = 'bgr8'
            msg.step = frame.shape[1] * 3
            pub.publish(msg)
        rate.sleep()

    cap.release()


if __name__ == '__main__':
    try:
        usb_camera()
    except rospy.ROSInterruptException:
        pass
