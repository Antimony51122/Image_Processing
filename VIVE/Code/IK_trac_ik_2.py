#!/usr/bin/env python

import sys
import rospy
import baxter_interface
from trac_ik_python.trac_ik import IK
import thread
import roslib
# ?? roslib/load_manifest()
import rospy
import math
import tf
# from sensor_msgs.msg import Joy
import geometry_msgs.msg
import time
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import baxter_external_devices
# import joy_listener


def callback(data):
	global grip_left, cal, passer
	passer = 0
	"""trig = 100 - (data.axes[0] * 100)
	if trig <= 90:
		trig = 90
	if trig >= 10:
		trig = 10
	print trig
	gripper.command_position(trig)
	#print data.buttons"""

	if cal == True:
		if data.buttons[3]:
			passer = 1
		else:
			passer = 0
	elif cal == False:
		None

	



def listener_thread():
	#rospy.init_node('node_1')
	rospy.Subscriber("/vive_right", Joy, callback)
	rospy.spin()

def rot_90_y(vector):
	rot_mat = np.matrix([[0,0,1], [0,1,0], [-1, 0, 0]])
	rot_vector = np.matmul(vector, rot_mat)
	return rot_vector

def scale(x, a, b, c, d):
	#If your number X falls between A and B, and you would like Y to fall 
	# between C and D, you can apply the following linear transform:

	# A = person lower bound
	# B = person upper bound

	# C = Baxter lower bound
	# D = Baxter upper bound

	# x = tf transform of controller to headset
	# return is the scaled deniro output
	return (x-a)/(b-a)*(d-c)+c

	# e.g. x_scaled = scale(trans_l[0], x_body_pos, x_extended, x_hug, x_forward)

"""
def main():

	listener = tf.TransformListener()
	rate = rospy.Rate(100.0)
	
	try:
		print 'trying'
		(trans_l,rot_l) = listener.lookupTransform('/left_controller','/hmd', rospy.Time(0))
		(trans_r,rot_r) = listener.lookupTransform('/right_controller','/hmd', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
			
	
	tf.TransformListener()
		quart_l = quaternion_from_euler(rot_l[0], rot_l[1], rot_l[2]) #wxyz
	quart_r = quaternion_from_euler(rot_r[0], rot_r[1], rot_r[2])
	#print "Left xyz = " + str(trans_l)
	#print "Left rot = " + str(rot_l)
	#print "Left quat = " + str(quart_l)
	#print "Right xyz = " + str(trans_r)
	#print "Right rot = " + str(rot_r)
	#print "Right quat = " + str(quart_r)
	time.sleep(0.2)
	return trans_l, trans_r, quart_l, quart_r
"""



if __name__ == '__main__':
	global grip_left, precal
	grip_left = None
	# rospy.init_node("IK_trac_ik")
	rospy.init_node('VIVE_listener')

	# thread.start_new_thread(listener_thread, ())

	urdf_str = rospy.get_param('/robot_description')
	ik_solver = IK(base_link="base", tip_link="left_gripper", urdf_string=urdf_str)
	
	lb, up = ik_solver.get_joint_limits()
	ik_solver.set_joint_limits([-1.70168, -2.147, -3.05418, -0.05, -3.059, -1.5708, -3.059],
					 [1.70168, 1.047, 3.05418, 2.618, 3.059, 2.094, 3.059])
	precal = True
	cal = True
	first_run = True

	left = baxter_interface.Limb('left')
	grip_left = baxter_interface.Gripper('left')
	lj = left.joint_names()
	left.set_joint_position_speed(0.3)
	# left.move_to_neutral(timeout=15.0)

	listener = tf.TransformListener()

	while not rospy.is_shutdown():
		if precal == True:
			left = baxter_interface.Limb('left')
			lj = left.joint_names()
			left.set_joint_position_speed(0.3)
			# print '*** Calibrating ***'
			# rospy.sleep(0.5)
			# c = raw_input("Press 'Enter' when the arm is in the correct position")
			precal = False
			print precal

		if cal == True:
			print '*** Calibrating ***'
			c = raw_input("Press 'Enter' when the arm is hugging the body")

			# while passer == 0:
			# 	time.sleep(0.3)

			try:
				# (cal_lt,cal_lq) = listener.lookupTransform('/sonar_ring','/base', rospy.Time(0))
				(cal_lt2,cal_lq2) = listener.lookupTransform('/hmd','/left_controller', rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			x_body_pos = cal_lt2[0]
			print x_body_pos

			c = raw_input("Press 'Enter' when the arm is fully extended forwards")
			try:
				# (cal_lt,cal_lq) = listener.lookupTransform('/sonar_ring','/base', rospy.Time(0))
				(cal_lt3,cal_lq2) = listener.lookupTransform('/hmd','/left_controller', rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			x_extended = cal_lt3[0]
			print x_extended
			c = raw_input("Press 'Enter' when the arm is fully extended upwards")

			try:
				# (cal_lt,cal_lq) = listener.lookupTransform('/sonar_ring','/base', rospy.Time(0))
				(cal_lt4,cal_lq2) = listener.lookupTransform('/hmd','/left_controller', rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			z_extended_up = cal_lt4[2]
			print z_extended_up
			c = raw_input("Press 'Enter' when the arm is fully extended downwards")

			try:
				# (cal_lt,cal_lq) = listener.lookupTransform('/sonar_ring','/base', rospy.Time(0))
				(cal_lt5,cal_lq2) = listener.lookupTransform('/hmd','/left_controller', rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			z_extended_down = cal_lt5[2]
			print z_extended_down

			c = raw_input("Press 'Enter' when the arm is fully extended left")

			try:
				# (cal_lt,cal_lq) = listener.lookupTransform('/sonar_ring','/base', rospy.Time(0))
				(cal_lt6,cal_lq2) = listener.lookupTransform('/hmd','/left_controller', rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			y_extended_left = cal_lt6[1]
			print y_extended_left

			c = raw_input("Press 'Enter' when the arm is fully extended right")

			try:
				# (cal_lt,cal_lq) = listener.lookupTransform('/sonar_ring','/base', rospy.Time(0))
				(cal_lt7,cal_lq2) = listener.lookupTransform('/hmd','/left_controller', rospy.Time(0))

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			y_extended_right = cal_lt7[1]
			print y_extended_right

			# BAXTER MOVEMENT BOUNDS
			x_forward = 1.2
			x_hug = 0.4
			x_range = x_forward - x_hug #0.9

			y_left = 1.45
			y_right = -0.3
			y_range = y_left - y_right #1.75

			z_up = 1.4
			z_down = 0.03
			z_range = z_up - z_down #1.37

			# print cal_lt
			print cal_lt2
			z_cal = -0.18 # cal_lt2[2] # = 0.25# -0.817 #trans_l[2]
			cal_fact = 0.817/1.5 - z_cal# 0.817-z_cal # Now we have the co ordinates of the vive relative to the base
			cal = False
			print cal
			print "***CALIBRATED***"
			c = raw_input("GO GO GO!!")

		try:
			# (trans_h, quart_h) = listener.lookupTransform('/hmd','/world', rospy.Time(0))

			(trans_l,quartl) = listener.lookupTransform('/left_controller','/hmd', rospy.Time(0))
			(trans_r,quartr) = listener.lookupTransform('/right_controller','/hmd', rospy.Time(0)) 

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		rot_l = euler_from_quaternion([quartl[0], quartl[1], quartl[2], quartl[3]])
		rot_r = euler_from_quaternion([quartr[0], quartr[1], quartr[2], quartr[3]])

		quart_l = quaternion_from_euler(1*rot_l[0], 1*rot_l[1] - np.pi/2 - np.pi/4, 1*rot_l[2])  # THIS COULD BE THE ISSUE
		quart_r = quaternion_from_euler(1*rot_r[0], 1*rot_r[1] - np.pi/2 -np.pi/4, 1*rot_r[2])

		# for i in range(len(trans_l)):
		trans_l[0] = trans_l[0]*-1
		trans_l[2] = trans_l[2]*-1

		# return (x-a)/(b-a)*(d-c)+c

		x_scaled = scale(trans_l[0], x_body_pos, x_extended, x_hug, x_forward)
		y_scaled = scale(trans_l[1], y_extended_left, y_extended_right, y_left, y_right)
		z_scaled = scale(trans_l[2], z_extended_down, z_extended_up, z_down, z_up)

		# trans_l[0] = trans_l[0]*1.5 # extend x axis
		# trans_r[0] = trans_r[0]*1.5
		# trans_l[1] = trans_l[1]*1.4 # extend y axis
		# trans_r[1] = trans_r[1]*1.4
		# trans_l[2] = trans_l[2]*1.5
		# trans_r[2] = 
		
		xl = x_scaled
		yl = y_scaled
		zl = z_scaled # + cal_fact

		print 'Target co-ordinates = ', xl, yl, zl

		Qwl = quart_l[0] #THIS COULD BE THE ISSUE, THE NUMBERS FOR THIS HERE
		Qxl = quart_l[1]
		Qyl = quart_l[2]
		Qzl = quart_l[3]
			
		bx = by = bz = 0.02 #brx = bry = brz = 9999.0  # We don't care about orientation
		brx = bry = brz = 9999.0
		current_position = left.joint_angles()

		temp = []
		dictlist = []
		for key, value in current_position.iteritems():
			temp = [key,value]
			dictlist.append(temp[1])
		# print current_position
		
		ordered_joints = [dictlist[5], dictlist[6], dictlist[3], dictlist[4], dictlist[0], dictlist[1], dictlist[2]]
		# print ordered_joints

		### SEEDING CURRENT JOINT POSITIONS INTO THE SOLVER
		seed_state = ordered_joints	
		# seed_state = [0,0,0,0,0,0,0]	
		# print seed_state
		


		# joints_from_ik1 = ik_solver.get_ik(seed_state, 0.6366221863461569, 0.8497028752234516, 0.05416354881648494, -0.3823911659266366, 0.9223729088465533, 0.023089273180701524, 0.04972020425543535)#, bx, by, bz, brx, bry, brz)  # QX, QY, QZ, QW , ,

		joints_from_ik1 = ik_solver.get_ik(seed_state, xl, yl, zl, Qwl, Qxl, Qyl, Qzl, bx, by, bz, brx, bry, brz)  # QX, QY, QZ, QW , ,

		# ik_solver.CartToJnt(qinit,
		# 					x, y, z,
		# 					rx, ry, rz, rw,
		# 					bx, by, bz,
		# 					brx, bry, brz)

		# print "IK solver uses link chain:"
		# print ik_solver.link_names
		print joints_from_ik1, 'Joint Solutions'
		# positions = dict(zip(left.joint_names(), list(joints_from_ik1)))
		# left.move_to_joint_positions(positions, timeout=10.0, threshold=0.008726646)

		# break

		if joints_from_ik1 == None:
			print 'passed'
			pass
		elif first_run == True:
			joints_from_ik = list(joints_from_ik1)
			positions = dict(zip(left.joint_names(), joints_from_ik))
		# positions = dict(zip(left.joint_names(), [-0.9804847644947284, -0.07547962270476974, 2.8779113859306897, 0.6242229914402619, -2.5616644372112836, 1.7232777704927458, -1.0106066157629539]))
			print 'moving?'
			# left.move_to_joint_positions(positions, timeout=3.0, threshold=0.008726646)
			print 'finished?'
			first_run = False
		elif joints_from_ik1 != None:
			joints_from_ik = list(joints_from_ik1)
			positions = dict(zip(left.joint_names(), joints_from_ik1))
			print 'moving?'
			# left.set_joint_positions(positions)
			print 'finished?'
		
		time.sleep(0.2)
		# rospy.sleep(5)

		# break

		# if joints_from_ik == None:
		# 	pass	
		# else:
		# 	positions = dict(zip(left.joint_names(), joints_from_ik))
		
		# 	left.set_joint_positions(positions)#, timeout=15.0, threshold=0.008726646)


		# This solver will now work for positions relative to the sonar ring. So we just need to =/- a distance between the vive and  		  headset from the Z axis in the solver, to scale the results accordingly. 

		# distance between base and sonar Ring is 1.015m or 1015mm or 101.5cm. The Z direction is the up/down direction of DENIRO. 
		# distance between horizontal position of hand and sonar ring is 437mm. This is along the Z axis also
		  
		# The user needs to hold their hands out horizontally. This will give a Z transform distance between the Vive and the arms. 		# Say for me it is 170mm=17cm=0.17m. We need to 'move' the Vive co-ordinates (0.437-0.17) down along the Z axis.
		
		# rostopic echo /tf | grep frame_id




