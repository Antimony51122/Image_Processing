# INSTRUCTIONS:
#---------------------
# Run baxter sim (roslaunch baxter_gazebo baxter_world.launch) or baxter, to start a master

# source ~/src/openvr/steamvr.sh to enable the vive devices
# Run htc-vive server and client (https://github.com/robosavvy/vive_ros)
#	roslaunch vive_ros server_vr.launch
#	roslaunch vive_ros vive.launch
# This starts a topic "/tf" which publishes the controller and hm tracking info


# Run kinect
# roslaunch kinect2_bridge kinect2_bridge.launch
# rosrun kinect2_viewer kinect2_viewer kinect2 sd cloud
# Run gen_vr_img.py - runs a compositor which creates hmd image from 3d point cloud
#-----------------

# TODO:
# - send commands from tracker to baxter successfully (in sim)
# - (openvr) get controller button state
# - (openvr) send image to HMD
#####################################################################

import sys
import time
import numpy as np
import signal
import roslib
import rospy
import tf
import baxter_interface

import ik_solver

from baxter_core_msgs.msg import JointCommand

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (
    SpawnModel,
    SetModelState,
    DeleteModel,
)


#####################################################################

USE_POSE = True
USE_SIM = False

#####################################################################

# def applyTransf(trans, rot):
#     T= tf.transformations.quaternion_matrix(rot)
#     T[0:3,3] = trans #[-trans[2], -trans[0], trans[1]]
#     rT1 = tf.transformations.rotation_matrix( -0.5*np.pi, (0,1,0))
#     # rT2 = tf.transformations.rotation_matrix( 0.5*np.pi, (0,0,1))
#     tT = tf.transformations.identity_matrix()
#     tT[0:3,3] = [0.1, 0, 1.8]
#     newT = T.dot(rT1)#.dot(rT2)
#     # print "\n\n"
#     # print T
#     # print newT
#     newtrans = newT[0:3,3] 
#     newrot = tf.transformations.quaternion_from_matrix(newT)
# 
#     return newtrans, newrot


def Vec2Pose(trans, rot):   
    # newtrans, newrot = applyTransf(trans, rot)
    newtrans, newrot = trans, rot
    position = Point( 
        x = newtrans[0], 
        y = newtrans[1], 
        z = newtrans[2] )
    orientation = Quaternion( 
        x = newrot[0], 
        y = newrot[1], 
        z = newrot[2], 
        w = newrot[3])
    pose = Pose(position=position, orientation=orientation)

    return pose

# change to add offset as param
def Vec2Pose_SIM(trans, rot):    
    # newtrans, newrot = applyTransf(trans, rot)
    newtrans, newrot = trans, rot
    position = Point( 
        x = newtrans[0] + 0.1 , 
        y = newtrans[1], 
        z = newtrans[2] + 1.8 )
    orientation = Quaternion( 
        x = newrot[0], 
        y = newrot[1], 
        z = newrot[2], 
        w = newrot[3])
    pose = Pose(position=position, orientation=orientation)

    return pose


def signal_handler(signal, frame):
        print('\nEXITING...')
        delete_target('target_left')
        delete_target('target_right')
        delete_target('target_hmd') 
        sys.exit(0)


def fixRot(quat):
    # convert quaternions to euler
    eul = tf.transformations.euler_from_quaternion(quat)
    # change euler and convert back to quaternions
    quat_fix = tf.transformations.quaternion_from_euler( -eul[2] + np.pi, -eul[0], eul[1] )

    return quat_fix


def publishIKdata(joint_vals, limb):
    # available control modes:
    # POSITION_MODE=1
    # VELOCITY_MODE=2
    # TORQUE_MODE=3
    # RAW_POSITION_MODE=4
    msg = JointCommand()
    msg.names = joint_vals.keys()
    msg.command = joint_vals.values()
    msg.mode = JointCommand.POSITION_MODE

    if limb=='left':
        pub_left.publish(msg)
    if limb=='right':
        pub_right.publish(msg)


#####################################################################
rospy.init_node("BVR_tracker2reference")

limb_left = baxter_interface.Limb("left")
limb_right = baxter_interface.Limb("right")

pub_left = rospy.Publisher('BVR_tracker2reference_topic_left', JointCommand, queue_size=1)
pub_right = rospy.Publisher('BVR_tracker2reference_topic_right', JointCommand, queue_size=1)

listener = tf.TransformListener()
rate = rospy.Rate(100)  # 100Hz

#####################################################################
# Simulation initialisation
if USE_SIM:
    REF_FRAME = 'world'
    spawn_target = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)#, persistent=True)
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)#, persistent=True)
    delete_target = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)#, persistent=True)
    # read target SDF
    # left is GREEN
    with open ('target_left.sdf', 'r') as target_file1: target_xml_left=target_file1.read().replace('\n', '')
    # right is BLUE
    with open ('target_right.sdf', 'r') as target_file2: target_xml_right=target_file2.read().replace('\n', '')
    # hmd is RED
    with open ('target_hmd.sdf', 'r') as target_file3: target_xml_hmd=target_file3.read().replace('\n', '')
    # show
    target_pose_left = Pose(position=Point(x=0,y=0,z=0), orientation=Quaternion(x=0,y=0,z=0,w=0))
    target_pose_right = Pose(position=Point(x=0,y=0,z=0), orientation=Quaternion(x=0,y=0,z=0,w=0))
    target_pose_hmd = Pose(position=Point(x=0,y=0,z=0), orientation=Quaternion(x=0,y=0,z=0,w=0))
    spawn_target('target_left', target_xml_left, '/', target_pose_left, REF_FRAME)
    spawn_target('target_right', target_xml_right, '/', target_pose_right, REF_FRAME)
    spawn_target('target_hmd', target_xml_hmd, '/', target_pose_hmd, REF_FRAME)

#####################################################################

signal.signal(signal.SIGINT, signal_handler)

while True:    
    flagL = 0
    flagR = 0
    flagH = 0

    # Get tracker pose
    print "---------------------------------------------------------------------------------------"
    try:
        (trans, rot) = listener.lookupTransform('/hmd', '/controller1', rospy.Time(0))
        trans_ctrl1 = [-trans[2], -trans[0], trans[1]] #[trans[0], trans[1], trans[2]]#trans#
        scale = 1.0
        trans_ctrl1 = [scale * trans_ctrl1[0] + 0.4, scale * trans_ctrl1[1], trans_ctrl1[2]]
        rot_ctrl1 = fixRot(rot)
        flagL = 1
        print "CTRL1_left: ", trans_ctrl1,"\n"
        # print "CTRL1: \n", tf.transformations.euler_from_quaternion(rot_ctrl1), "\n"
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    	print "Waiting for CONTROLLER 1 (LEFT)"
        
    try:
        (trans, rot) = listener.lookupTransform('/hmd', '/controller2', rospy.Time(0))
        trans_ctrl2 = [-trans[2], -trans[0], trans[1]]
        trans_ctrl2 = [scale * trans_ctrl2[0] + 0.4, scale * trans_ctrl2[1], trans_ctrl2[2]]
        rot_ctrl2 = fixRot(rot)
        flagR = 1
        print "CTRL2_right: ", trans_ctrl2,"\n"
        # print "CTRL2: \n", tf.transformations.euler_from_quaternion(rot_ctrl2), "\n"
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    	print "Waiting for CONTROLLER 2 (RIGHT)"
        
    try:
        (trans_hmd, rot_hmd) = listener.lookupTransform('/hmd', '/hmd', rospy.Time(0))
        flagH = 1
        #print "HMD: ", trans_hmd,"\n"
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    	print "Waiting for HMD"


    # Display the desired poses in SIM
    if USE_SIM:
        if flagL:
            target_pose_left = Vec2Pose_SIM(trans_ctrl1, rot_ctrl1)
            target_state_left = ModelState(model_name='target_left', pose=target_pose_left, reference_frame=REF_FRAME)
            set_model_state(target_state_left)

        if flagR:
            target_pose_right= Vec2Pose_SIM(trans_ctrl2, rot_ctrl2)
            target_state_right = ModelState(model_name='target_right', pose=target_pose_right, reference_frame=REF_FRAME)
            set_model_state(target_state_right)

        if flagH:
            target_pose_hmd= Vec2Pose_SIM(trans_hmd, rot_hmd)
            target_state_hmd = ModelState(model_name='target_hmd', pose=target_pose_hmd, reference_frame=REF_FRAME)
            set_model_state(target_state_hmd)


    # Include POSE information
    if not USE_POSE:
        try:
            (tl, rot_wrist_left) = listener.lookupTransform('/sonar_ring', '/left_wrist', rospy.Time(0))
            print "LIMB1_left: ", tl, "\n"#[-tl[2], -tl[0], tl[1]]
            rot_ctrl1 = rot_wrist_left
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (tr, rot_wrist_right) = listener.lookupTransform('/sonar_ring', '/right_wrist', rospy.Time(0))
            print "LIMB2_right: ", tr, "\n"
            rot_ctrl2 = rot_wrist_right
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


    # Transfer Cartesian to joints
    if flagL:
        # get joint angles
        joint_values_left = ik_solver.ik_solve('left', Vec2Pose(trans_ctrl1, rot_ctrl1), limb_left.joint_angles())
        # publish IK data
        if not (joint_values_left==1 or joint_values_left==-1):
            publishIKdata(joint_values_left, 'left')
        else:
            publishIKdata(limb_left.joint_angles(), 'left')
    else:
        publishIKdata(limb_left.joint_angles(), 'left')        

    if flagR:
        # get joint angles
        joint_values_right = ik_solver.ik_solve('right', Vec2Pose(trans_ctrl2, rot_ctrl2), limb_right.joint_angles())
        # publish IK data
        if not (joint_values_right==1 or joint_values_right==-1):
            publishIKdata(joint_values_right, 'right')
        else:
            publishIKdata(limb_right.joint_angles(), 'right')
    else:
        publishIKdata(limb_right.joint_angles(), 'right')    


    rate.sleep()

    print "check"
