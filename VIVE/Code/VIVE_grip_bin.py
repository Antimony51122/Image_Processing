#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import baxter_external_devices
import baxter_interface
import time

def callback(data):
	"""trig = 100 - (data.axes[0] * 100)
	if trig <= 90:
		trig = 90
	if trig >= 10:
		trig = 10
	print trig
	gripper.command_position(trig)
	#print data.buttons"""
	if data.buttons[0]:
		grip_right.close()
		print "Closing..."
	else:
		grip_right.open()
		print "Open"

def listener():
	#rospy.init_node('node_1')
	rospy.Subscriber("/vive_right", Joy, callback)
	rospy.spin()


if __name__ == '__main__':
	rospy.init_node('VIVE_Grip')
	grip_right = baxter_interface.Gripper('right')
	grip_right.calibrate()
	listener()
	