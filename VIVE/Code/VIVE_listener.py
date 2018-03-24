import roslib
# ?? roslib/load_manifest()
import rospy
import math
import tf
import geometry_msgs.msg
import time
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':

    rospy.init_node('VIVE_listener')

    listener = tf.TransformListener()

    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('spawn', ##turtlesim.srv.spawn)
    #spawner (4, 2, 0)

    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():

        try:
            (trans_l,rot_l) = listener.lookupTransform('/left_controller','/hmd', rospy.Time(0))
            (trans_r,rot_r) = listener.lookupTransform('/right_controller','/hmd', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        tf.TransformListener()
        quart_l = quaternion_from_euler(rot_l[0], rot_l[1], rot_l[2])
        quart_r = quaternion_from_euler(rot_r[0], rot_r[1], rot_r[2])
        #print "Left xyz = " + str(trans_l)
        #print "Left rot = " + str(rot_l)
        #print "Left quat = " + str(quart_l)
        print "Right xyz = " + str(trans_r)
        #print "Right rot = " + str(rot_r)
        #print "Right quat = " + str(quart_r)

        time.sleep(0.2)













