#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
import struct

def diff_drive() -> None:
    rospy.init_node("diff_drive")
    number = rospy.get_param("~number", 0)
    ser = serial.Serial('/dev/ttyACM1', 115200)
    print(ser.name)

    def twist_callback(message: Twist) -> None:
        lin_vel = message.linear.x
        ang_vel = message.angular.z
        ser.write(struct.pack('<BBffB', 0xAB, 0xCD, lin_vel, ang_vel, 0xEF))

    cmd_vel_topic = f"cmd_vel_{number}"
    rospy.Subscriber(cmd_vel_topic, Twist, twist_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        diff_drive()
    except rospy.ROSInterruptException:
        pass
