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

class image_converter:

    def __init__(self):
        """publishing to rostopic"""
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.image_pub1 = rospy.Publisher("image_topic_3", Image)

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
        # (rows, cols, channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (50, 50), 10, 255)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners = cv2.goodFeaturesToTrack(gray, 100, 0.01, 15)

        # returns
        for corner in corners:
            x, y = corner[0]
            x = int(x)
            y = int(y)
            cv2.rectangle(cv_image, (x - 10, y - 10), (x + 10, y + 10), (0, 255, 0), 2)

        cv2.imshow("Corners Found", cv_image)

        cv2.waitKey(3)
        # print('cv')

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

