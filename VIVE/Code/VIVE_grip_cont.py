#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import baxter_external_devices
import baxter_interface
import time
def right_callback(data):
    trig = 100 - (data.axes[0] * 100)
    if trig >= 90:
        trig = 90
    if trig <= 10:
        trig = 10
    print type(trig)
    grip_right.command_position(trig)

def left_callback(data):
    trig = 100 - (data.axes[0] * 100)
    if trig >= 90:
        trig = 90
    if trig <= 10:
        trig = 10
    print trig
    grip_left.command_position(trig)

def listener():
    #rospy.init_node('node_1')
    rospy.Subscriber("/vive_right", Joy, right_callback)
    rospy.Subscriber("/vive_left", Joy, left_callback)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('VIVE_Grip')
    grip_left = baxter_interface.Gripper('left')
    grip_right = baxter_interface.Gripper('right')
    grip_left.calibrate()
    grip_right.calibrate()
    listener()

"""
Data.axes (type: list)

data.axes[0] = Trigger squeeze amount, 0 to 1, type: float
data.axes[1] = Trackpad touch x axis, -1 to 1, type: float
data.axes[2] = Trackpad touch y axis, -1 to 1, type: float

data.buttons (type: list)

data.buttons[0] = Trigger
data.buttons[1] = Trackpad touch
data.buttons[2] = Trackpad click
data.buttons[3] = Menu button
data.buttons[4] = Squeeze
All Binary
"""
