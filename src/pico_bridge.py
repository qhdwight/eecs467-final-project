#!/usr/bin/env python3

import struct
from pathlib import Path

import rospy
import serial
from geometry_msgs.msg import Twist
from hockey_cup.msg import BallControl
from hockey_cup.msg import WheelVelocities

MOTOR_CMD_TIMEOUT = 1


def get_highest_dev(prefix: str):
    sorted_paths = []
    for i in range(10):
        path = Path(f"/dev/{prefix}_{i}")
        if path.exists():
            sorted_paths.append(str(path))
    sorted_paths.sort()
    rospy.loginfo(f"Found {prefix} devices: {sorted_paths}")
    return sorted_paths[-1]


def pico_bridge() -> None:
    rospy.init_node("pico_bridge")
    number = rospy.get_param("~number", 0)
    drive_ser = serial.Serial(get_highest_dev('pico_drive'), 115200)
    ball_ser = serial.Serial(get_highest_dev('pico_ball'), 115200)
    rospy.loginfo(drive_ser)
    rospy.loginfo(ball_ser)

    def send_motor_cmd(linear_vel, angular_vel):
        drive_ser.write(struct.pack('<BBffB', 0xAB, 0xCD, linear_vel, angular_vel, 0xEF))
        rospy.loginfo("sending drive cmd")

    def send_drive_joy_cmd(left, right):
        drive_ser.write(struct.pack('<BBffB', 0xAB, 0x89, left, right, 0xEF))
        rospy.loginfo("sending joystick drive cmd")

    def send_ball_cmd(intake_cmd, shoot_cmd):
        ball_ser.write(struct.pack('<BBfBB', 0xAB, 0xCD, intake_cmd, shoot_cmd, 0xEF))
        rospy.loginfo('sending ball cmd')

    def timer_cb(_):
        send_motor_cmd(0, 0)
        rospy.loginfo("Timed out, stopping bot")

    timer = rospy.Timer(rospy.Duration(MOTOR_CMD_TIMEOUT), timer_cb, oneshot=True)

    def twist_callback(message: Twist) -> None:
        # Forward cmd to pico over serial
        lin_vel = message.linear.x
        ang_vel = message.angular.z
        send_motor_cmd(lin_vel, ang_vel)

        # Reset timer
        nonlocal timer
        timer.shutdown()
        timer = rospy.Timer(rospy.Duration(MOTOR_CMD_TIMEOUT), timer_cb, oneshot=True)

    def ball_callback(message: BallControl) -> None:
        # Forward cmd to pico over serial
        intake_cmd = message.top_roller
        shoot_cmd = message.shoot_ball
        send_ball_cmd(intake_cmd, shoot_cmd)

    def drive_joy_callback(message: WheelVelocities) -> None:
        send_drive_joy_cmd(message.left, message.right)

    cmd_vel_topic = f"cmd_vel_{number}"
    cmd_joy_drive_topic = f"cmd_joy_drive_{number}"
    cmd_ball_topic = f"cmd_ball_{number}"
    rospy.Subscriber(cmd_vel_topic, Twist, twist_callback)
    rospy.Subscriber(cmd_joy_drive_topic, WheelVelocities, drive_joy_callback)
    rospy.Subscriber(cmd_ball_topic, BallControl, ball_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        pico_bridge()
    except rospy.ROSInterruptException:
        pass
