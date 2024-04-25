#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial
import struct
import time

from hockey_cup.msg import BallControl

MOTOR_CMD_TIMEOUT = 1

def pico_bridge() -> None:
    rospy.init_node("pico_bridge")
    number = rospy.get_param("~number", 0)
    drive_ser = serial.Serial('/dev/ttyACM2', 115200)
    ball_ser = serial.Serial('/dev/ttyACM3', 115200)
    print(drive_ser)
    print(ball_ser)

    def send_motor_cmd(linear_vel, angular_vel):
        drive_ser.write(struct.pack('<BBffB', 0xAB, 0xCD, linear_vel, angular_vel, 0xEF))
        print("sending drive cmd")
    
    def send_ball_cmd(intake_cmd, shoot_cmd):
        ball_ser.write(struct.pack('<BBBBB', 0xAB, 0xCD, intake_cmd, shoot_cmd, 0xEF))
        print('sending ball cmd')

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
    
    def ball_callback(message: BallControl) -> None:
        # Forward cmd to pico over serial
        intake_cmd = message.top_roller
        shoot_cmd = message.shoot_ball
        send_ball_cmd(intake_cmd, shoot_cmd)

    cmd_vel_topic = f"cmd_vel_{number}"
    cmd_ball_topic = f"cmd_ball_{number}"
    rospy.Subscriber(cmd_vel_topic, Twist, twist_callback)
    rospy.Subscriber(cmd_ball_topic, BallControl, ball_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        pico_bridge()
    except rospy.ROSInterruptException:
        pass
