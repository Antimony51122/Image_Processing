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
import thread
from sensor_msgs.msg import Joy
# import baxter_external_devices
# import baxter_interface
import time

"""imports typically for ssd pre-trained data"""
import torch
from torch.autograd import Variable
from data import BaseTransform, VOC_CLASSES as labelmap
from ssd import build_ssd
import imageio

"""from Lynda tutorial"""
import random

# def find_marker(image):
#     # convert the image to grayscale, blur it, and detect edges
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     gray = cv2.GaussianBlur(gray, (5, 5), 0)
#     edged = cv2.Canny(gray, 35, 125)
#
#     # find the contours in the edged image and keep the largest one;
#     # we'll assume that this is our piece of paper in the image
#     (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
#     c = max(cnts, key=cv2.contourArea)
#
#     # compute the bounding box of the of the paper region and return it
#     return cv2.minAreaRect(c)
#
#
# def distance_to_camera(knownWidth, focalLength, perWidth):
#     # compute and return the distance from the maker to the camera
#     return (knownWidth * focalLength) / perWidth
#
# # initialize the known distance from the camera to the object, which
# # in this case is 24 inches
# KNOWN_DISTANCE = 24.0
#
# # initialize the known object width, which in this case, the piece of
# # paper is 11 inches wide
# KNOWN_WIDTH = 11.0

"""face detection"""
path = "haarcascade_frontalface_default.xml"

class image_converter():

    def __init__(self):
        """publishing to rostopic"""
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.image_pub_b = rospy.Publisher("image_topic_3", Image, queue_size=10)

        self.bridge = CvBridge()

        # check the channel (1st argument of Subcriber()) being subscribed
        self.image_sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        # self.image_sub_rgb = rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.callback)  # temporary testing
        # self.image_sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)
        # self.image_sub_ir = rospy.Subscriber("/camera/ir/image_raw", Image, self.callback)


    """callback to rgb camera"""
    def callback(self, data):
        """convert ROS image to OpenCV image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        """
        below are image processing
        """

        """face detection"""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        face_cascade = cv2.CascadeClassifier(path)

        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=5, minSize=(40, 40))
        print(len(faces))

        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # cv2.imshow("Image", cv_image)

        """canny edge detection"""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([30, 150, 50])
        upper_red = np.array([255, 255, 180])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        edges = cv2.Canny(cv_image, 100, 200)
        # cv2.imshow('Edges', edges)

        """depth"""
        # marker = find_marker(cv_image)
        # focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
        # print(marker)

        cv2.waitKey(3)

        """convert OpenCV to ROS image and publish"""
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(edges, "8UC1"))
            self.image_pub_b.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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

    """listener of hand controller"""
    # def callback_listener(self, data):
    #     """convert OpenCV to ROS image and publish by pressing hand controller button"""
    #     try:
    #         if data.buttons[2] == 1:
    #             self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
    #         elif data.buttons[2] == 0:
    #             self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.edges, "bgr8"))
    #     except CvBridgeError as e:
    #         print(e)

    # def listener_thread(self):
    #     # rospy.init_node('node_1')
    #     rospy.Subscriber("/vive_right", Joy, self.callback_listener)
    #     rospy.Subscriber("/vive_left", Joy, self.callback_listener)
    #
    #     rospy.spin()

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # thread.start_new_thread(image_converter.listener_thread, ())
    print('listening')

    main(sys.argv)



