#!/usr/bin/env python3

from pyPS4Controller.controller import Controller
import numpy as np
import rospy
from hockey_cup.msg import WheelVelocities, BallControl

L_DOWN_THRESHOLD = 2580
L_UP_THRESHOLD = -259

R_DOWN_THRESHOLD = 516
R_UP_THRESHOLD = -259

MAX_LIMIT = np.power(2, 15) - 1
MIN_LIMIT = -np.power(2, 15) + 1

latest_left = 0
latest_right = 0

rospy.init_node("teleop")
number = rospy.get_param("~number", 0)
cmd_joy_drive_topic = f"cmd_joy_drive_{number}"
cmd_ball_topic = f"cmd_ball_{number}"
drive_pub = rospy.Publisher(cmd_joy_drive_topic, WheelVelocities, queue_size=1)
ball_pub = rospy.Publisher(cmd_ball_topic, BallControl, queue_size=1)

def send_drive_cmd(left, right):
    print('left: ', left, ' right: ', right)
    drive_pub.publish(WheelVelocities(left, right))
def send_ball_cmd(intake, shoot):
    if shoot > 0:
        print('Shooting')
    else:
        print('Intake cmd: ', intake)
    ball_pub.publish(BallControl(intake, shoot))

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.latest_left = 0
        self.latest_right = 0

    def on_L3_up(self, val):
        cmd = 0 if val > L_UP_THRESHOLD else (val - L_UP_THRESHOLD) / (MIN_LIMIT - L_UP_THRESHOLD)
        send_drive_cmd(cmd, self.latest_right)
        self.latest_left = cmd
    def on_L3_down(self, val):
        cmd = 0 if val < L_DOWN_THRESHOLD else -(val - L_DOWN_THRESHOLD) / (MAX_LIMIT - L_DOWN_THRESHOLD)
        send_drive_cmd(cmd, self.latest_right)
        self.latest_left = cmd
    def on_R3_up(self, val):
        cmd = 0 if val > R_UP_THRESHOLD else (val - R_UP_THRESHOLD) / (MIN_LIMIT - R_UP_THRESHOLD)
        send_drive_cmd(self.latest_left, cmd)
        self.latest_right = cmd
    def on_R3_down(self, val):
        cmd = 0 if val < R_DOWN_THRESHOLD else -(val - R_DOWN_THRESHOLD) / (MAX_LIMIT - R_DOWN_THRESHOLD)
        send_drive_cmd(self.latest_left, cmd)
        self.latest_right = cmd

    def on_L2_release(self):
        send_ball_cmd(0, 0)
    def on_R1_release(self):
        send_ball_cmd(0, 0)
    def on_L2_press(self, val):
        cmd = (val - MIN_LIMIT) / (MAX_LIMIT - MIN_LIMIT)
        send_ball_cmd(cmd, 0)
    def on_R2_press(self, val):
        cmd = -(val - MIN_LIMIT) / (MAX_LIMIT - MIN_LIMIT)
        send_ball_cmd(cmd, 0)

    def on_x_press(self):
        send_ball_cmd(0, 1)


    def on_x_release(self):
        ...
    def on_triangle_press(self):
        ...
    def on_triangle_release(self):
        ...
    def on_circle_press(self):
        ...
    def on_circle_release(self):
        ...
    def on_square_press(self):
        ...
    def on_square_release(self):
        ...
    def on_L1_press(self):
        ...
    def on_L1_release(self):
        ...
    def on_R1_press(self):
        ...
    def on_R2_release(self):
        ...
    def on_up_arrow_press(self):
        ...
    def on_up_down_arrow_release(self):
        ...
    def on_down_arrow_press(self):
        ...
    def on_left_arrow_press(self):
        ...
    def on_left_right_arrow_release(self):
        ...
    def on_right_arrow_press(self):
        ...
    def on_L3_x_at_rest(self):
        ... 
    def on_L3_y_at_rest(self):
        ...  
    def on_L3_press(self):
        ...
    def on_L3_release(self):
        ...
    def on_R3_x_at_rest(self):
        ...
    def on_R3_y_at_rest(self):
        ...
    def on_R3_press(self):
        ...
    def on_R3_release(self):
        ...
    def on_options_press(self):
        ...
    def on_options_release(self):
        ...
    def on_share_press(self):
        ...
    def on_share_release(self):
        ...
    def on_playstation_on_buttn_press(self):
        ...
    def on_playstatidon_button_release(self):
        ... 
    def on_L3_left(self, val):
        ...
    def on_L3_right(self, val):
        ...
    def on_R3_left(self, val):
        ...
    def on_R3_right(self, val):
        ...

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()
rospy.spin()
