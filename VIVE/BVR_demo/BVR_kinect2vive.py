#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import time

class image_converter:

  def __init__(self):
    # self.image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.callback, queue_size=1)
    #self.image_sub = rospy.Subscriber("/kinect2/hd/image_color", Image, self.callback, queue_size=1)
    #self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/hd/image_color/compressed", CompressedImage, self.callback, queue_size=1)
    self.offset = 70 #200 #245
    self.offset_left_right = 100 #120
    self.is_ok = True

  def callback(self, data):
    vive_image = np.zeros((1200, 2160, 3), dtype=np.uint8)
    #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    np_arr = np.fromstring(data.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR) # (1080, 1920, 3)
    scale = 0.7
    image = cv2.resize(cv_image, None, fx=scale, fy=scale, interpolation = cv2.INTER_LINEAR) #cv2.INTER_CUBIC)
    #cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
    # print(image.(), image.max())
    shape = image.shape
    #print(image.shape)
    border = (1200 - shape[0]) / 2
    #print(self.offset+self.offset_left_right, self.offset+1080+self.offset_left_right)
    vive_image[50+border:50+border+shape[0], 0:1080]    = image[:, self.offset+0:self.offset+1080]
    vive_image[50+border:50+border+shape[0], 1080:2160] = image[:, self.offset+self.offset_left_right:self.offset+1080+self.offset_left_right]

    cv2.imshow("Vive", vive_image)
    key = cv2.waitKey(1)
    if key == 1048603:
      self.is_ok = False

def main(args):
  rospy.init_node("head_camera_to_vive", anonymous=False)
  ic = image_converter()
  cv2.namedWindow("Vive", cv2.WND_PROP_FULLSCREEN)          
  cv2.setWindowProperty("Vive", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
  while not rospy.is_shutdown() and ic.is_ok:
    time.sleep(0.1)

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
