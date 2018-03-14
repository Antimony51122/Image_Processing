#!/usr/bin/env python
from __future__ import print_function

# import roslib
# roslib.load_manifest('roscreate-pkg')
# roslib.load_manifest('roscreate-pkg')
# roslib.load_manifest('roscreate-pkg')  # various different packages

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image)

        self.bridge = CvBridge()
        self.image_sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_rgb)
        self.image_sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback_depth)
        self.image_sub_ir = rospy.Subscriber("/camera/ir/image_raw", Image, self.callback_ir)
        # check the channel (1st argument of Subcriber()) being subscribed

    def callback_rgb(self, data):
        """convert ROS image to OpenCV image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        """image processing"""
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50, 50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        print('cv')

        """convert OpenCV to ROS image and publish"""
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            print('cv')
        except CvBridgeError as e:
            print(e)

    def callback_depth(self, data):
        """convert ROS image to OpenCV image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # leave for now but depth cannot do 'bgr8'
        except CvBridgeError as e:
            print(e)

    def callback_ir(self, data):
        """convert ROS image to OpenCV image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # leave for now but depth cannot do 'bgr8'
        except CvBridgeError as e:
            print(e)

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

