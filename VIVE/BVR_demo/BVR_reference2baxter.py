
#####################################################################
#
# Listen to the /tracker2cartesian node's topic: /tracker2cartesian
# and implement the given joint angles
#
#####################################################################

import time
import numpy as np
import roslib
import rospy
import baxter_interface

from std_msgs.msg import UInt64MultiArray
from baxter_core_msgs.msg import JointCommand

##################################################################### 
# available control modes:
# POSITION_MODE=1
# VELOCITY_MODE=2
# TORQUE_MODE=3
# RAW_POSITION_MODE=4
jvals_left, jvals_right, ctrl_mode = None, None, None
ctrl_mode = JointCommand.POSITION_MODE
ctrl_btns_left=[]
ctrl_btns_right=[]
ctrl_btns_left.append(0)
ctrl_btns_right.append(0)

def setAngles(joint_angles, limb):
    global start_angles_left, start_angles_right
    if limb=='left':
        start_angles_left = joint_angles
    elif limb=='right':
        start_angles_right = joint_angles

def callback(data, args):
    global jvals_left, jvals_right, ctrl_mode
    # unpack data
    ctrl_mode = data.mode
    if args=='left':
        jvals_left = dict(zip(data.names, data.command))  
    elif args=='right':
        jvals_right = dict(zip(data.names, data.command))  

def callback_btn(data):
    global ctrl_btns_left, ctrl_btns_right
    ctrl_btns_left = bin(data.data[0])[2:].zfill(34)
    ctrl_btns_right = bin(data.data[1])[2:].zfill(34)

    # print ctrl_btns_left[0], ctrl_btns_right[0]

def _main_():
    # # LEFT
    if ctrl_btns_left[0]=='1':
    # if True:
        # print 'LEFT'
        if ctrl_mode == JointCommand.POSITION_MODE:        
            limb_left.set_joint_positions(jvals_left)
            print "\nmoving left...."
        elif mode == JointCommand.TORQUE_MODE: 
            for joint in start_angles_left.keys():
                # spring portion
                cmd[joint] = springs[joint] * (start_angles_left[joint] - cur_pos[joint])
                # damping portion
                cmd[joint] -= damping[joint] * cur_vel[joint]
                limb_left.set_joint_torques(cmd)
        # execute grippers
        if ctrl_btns_left[1]=='1':
           grip_left.close()
        else:
           grip_left.open()
    # RIGHT
    if ctrl_btns_right[0]=='1':
    # if True:
        if ctrl_mode == JointCommand.POSITION_MODE:        
            limb_right.set_joint_positions(jvals_right)
            print "\nmoving right...."
        elif ctrl_mode == JointCommand.TORQUE_MODE: 
            for joint in start_angles_right.keys():
                # spring portion
                cmd[joint] = springs[joint] * (start_angles_right[joint] - cur_pos[joint])
                # damping portion
                cmd[joint] -= damping[joint] * cur_vel[joint]
                limb_right.set_joint_torques(cmd)
        # execute grippers
        if ctrl_btns_right[1]=='1':
           grip_right.close()
        else:
           grip_right.open()

        
#####################################################################
rospy.init_node("BVR_reference2baxter", anonymous=False)

#####################################################################
# Baxter initialisation
limb_left = baxter_interface.Limb("left")
limb_right = baxter_interface.Limb("right")

grip_left = baxter_interface.Gripper("left")
grip_right = baxter_interface.Gripper("right")

limb_left.set_joint_position_speed(0.5)
limb_right.set_joint_position_speed(0.5)

print "\nMOVEMENTS ENABLED\n"
# limb_left.move_to_neutral()
# limb_right.move_to_neutral()
# time.sleep(5)
rate = rospy.Rate(1000) 
#####################################################################

rospy.Subscriber("BVR_tracker2reference_topic_left", JointCommand, callback, 'left', queue_size=1)
rospy.Subscriber("BVR_tracker2reference_topic_right", JointCommand, callback, 'right', queue_size=1)
rospy.Subscriber("BVR_vive_buttons_topic", UInt64MultiArray, callback_btn, queue_size=1)

grip_left.calibrate()
grip_right.calibrate()

while not rospy.is_shutdown():
    _main_()
    print "---------------------"
    print 'left_btn: ',ctrl_btns_left[0], '\nright_btn: ',ctrl_btns_right[0]
    rate.sleep()

# rospy.spin()