#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
import struct

MOTOR_CMD_TIMEOUT = 1

def diff_drive() -> None:
    rospy.init_node("pico_bridge")
    number = rospy.get_param("~number", 0)
    ser = serial.Serial('/dev/ttyACM1', 115200)
    print(ser.name)

    def send_motor_cmd(linear_vel, angular_vel):
        ser.write(struct.pack('<BBffB', 0xAB, 0xCD, linear_vel, angular_vel, 0xEF))
        print("sending")

    def timer_cb(_):
        send_motor_cmd(0, 0)
        print("Timed out, stopping bot")

    timer = rospy.Timer(rospy.Duration(MOTOR_CMD_TIMEOUT), timer_cb)

    def twist_callback(message: Twist) -> None:
        # Forward cmd to pico over serial
        lin_vel = message.linear.x
        ang_vel = message.angular.z
        send_motor_cmd(lin_vel, ang_vel)

        # Reset timer
        nonlocal timer
        timer.shutdown()
        timer = rospy.Timer(rospy.Duration(MOTOR_CMD_TIMEOUT), timer_cb)

    cmd_vel_topic = f"cmd_vel_{number}"
    rospy.Subscriber(cmd_vel_topic, Twist, twist_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        diff_drive()
    except rospy.ROSInterruptException:
        pass
