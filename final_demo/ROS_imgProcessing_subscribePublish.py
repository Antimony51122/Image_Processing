<<<<<<< HEAD
from __future__ import print_function

# import roslib
# roslib.load_manifest('roscreate-pkg')
# roslib.load_manifest('roscreate-pkg')
# roslib.load_manifest('roscreate-pkg')  # various different packages

import cv2
import numpy as np

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# the following imports for the hand grip controller
from sensor_msgs.msg import Joy
import baxter_external_devices
import baxter_interface
import time

"""imports typically for ssd pre-trained data"""
import torch
from torch.autograd import Variable
from data import BaseTransform, VOC_CLASSES as labelmap
from ssd import build_ssd
import imageio




class image_converter:

    def __init__(self):
        """publishing to rostopic"""
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        # self.image_pub_depth = rospy.Publisher("image_topic_3", Image)

        self.bridge = CvBridge()

        # check the channel (1st argument of Subcriber()) being subscribed
        self.image_sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_rgb)
        # self.image_sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback_depth)
        # self.image_sub_ir = rospy.Subscriber("/camera/ir/image_raw", Image, self.callback_ir)


    """callback to rgb camera"""
    def callback_rgb(self, data):
        """convert ROS image to OpenCV image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        """image processing"""
        # simple circle drawing
        # (rows, cols, channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (50, 50), 10, 255)



        cv2.waitKey(3)

        """convert OpenCV to ROS image and publish"""
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            print('cv image has been published')
        except CvBridgeError as e:
            print(e)

    # """callback to depth camera"""
    # def callback_depth(self, data):
    #     """convert ROS image to OpenCV image"""
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # remember depth cannot convert to 'bgr8'
    #     except CvBridgeError as e:
    #         print(e)
    #
    # """callback to ir camera"""
    # def callback_ir(self, data):
    #     """convert ROS image to OpenCV image"""
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # remember depth cannot convert to 'bgr8'
    #     except CvBridgeError as e:
    #         print(e)

    def right_callback(self, data):
        if data.buttons[2] == 1:  # pressed


        elif data.buttons[2] == 0:  # not pressed

        pass

    def left_callback(self, data):
        pass

    def listener(self):
        # rospy.init_node('node_1')
        rospy.Subscriber("/vive_right", Joy, self.right_callback)
        rospy.Subscriber("/vive_left", Joy, self.left_callback)

        rospy.spin()

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


