#!/usr/bin/env python

import numpy as np
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
import time

previous_time = time.time()
def callback(msg):
  global is_ok, previous_time

  encoded_image = np.fromstring(msg.data, np.uint8)
  cameras_image[:,:,:] = cv2.imdecode(encoded_image, cv2.CV_LOAD_IMAGE_COLOR)

  border = (vive_height - cameras_height_scaled) / 2
  left_image_scaled[:] = cv2.resize(left_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR) #cv2.INTER_CUBIC
  right_image_scaled[:] = cv2.resize(right_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR) #cv2.INTER_CUBIC
  vive_image[50+border:50+border+cameras_height_scaled, :vive_width] = left_image_scaled[:,:-offset]
  vive_image[50+border:50+border+cameras_height_scaled, vive_width:] = right_image_scaled[:,offset:]

  cv2.imshow("Vive", vive_image)
  if cv2.waitKey(1) == 1048603 or rospy.is_shutdown():
    is_ok = False

  new_time = time.time()
  print 1.0 / (new_time - previous_time)
  previous_time = new_time

image_sub = rospy.Subscriber("/cameras/eyes/compressed", CompressedImage, callback, queue_size=1)
offset = 50 #50
is_ok = True

cameras_width = 1920
cameras_height = 1080
vive_width = 1080
vive_height = 1200
cameras_image = np.empty((cameras_height, cameras_width*2, 3), dtype=np.uint8)
left_image = cameras_image[:,:cameras_width,:]
right_image = cameras_image[:,cameras_width:,:]
cameras_width_scaled = vive_width+offset
scale = float(cameras_width_scaled)/float(cameras_width)
cameras_height_scaled = int(round(scale*cameras_height))
left_image_scaled = np.empty((cameras_height_scaled, cameras_width_scaled, 3), dtype=np.uint8)
right_image_scaled = np.empty((cameras_height_scaled, cameras_width_scaled, 3), dtype=np.uint8)
vive_image = np.zeros((vive_height, vive_width*2, 3), dtype=np.uint8)

def main(args):
  rospy.init_node("baxter_eyes_to_vive", anonymous=False)
  cv2.namedWindow("Vive", cv2.WND_PROP_FULLSCREEN)          
  cv2.setWindowProperty("Vive", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
  while is_ok:
    time.sleep(0.1)

  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
