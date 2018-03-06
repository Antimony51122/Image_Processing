from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2

#dont forget to $ roscore and  $ roslaunch openni2_launch openni2.launch
#https://answers.ros.org/question/219029/getting-depth-information-from-point-using-python/ FOR MULTPLE INPUTS

predictor_path = "/home/robin/DE3-ROB1-FEEDING/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/mouthxy", String, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.callback, queue_size =1)
    self.image_depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.callback_d,queue_size =1)
  
  def callback_d(self,data):
    try:
      depth_image_raw = self.bridge.imgmsg_to_cv2(data, "passthrough")

      depth_image = depth_image_raw.astype(np.uint8)
      # depth_image = np.asarray(depth_image_raw)


    except CvBridgeError as e:
      print(e)
    print(depth_image[0][0])

    cv2.imshow("Depth window", depth_image_raw)
    cv2.waitKey(1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    rects = detector(gray, 1)

    for (i, rect) in enumerate(rects):
      shape = predictor(gray, rect)
      shape = face_utils.shape_to_np(shape)
      features = face_utils.FACIAL_LANDMARKS_IDXS.items()
      mouth = features[0]
      points = shape[mouth[1][0]:mouth[1][1]]
      # for (x,y) in points: 
      #   cv2.circle(cv_image, (x, y), 1, (0, 0, 255), -1)

      inside_points = shape[60:68]
      mouth_top = shape[62]
      mouth_bottom = shape[66]
      mouth_center_x = mouth_bottom[0] +(mouth_top[0]-mouth_bottom[0])/2
      mouth_center_y = mouth_bottom[1] +(mouth_top[1]-mouth_bottom[1])/2
      cv2.circle(cv_image, (mouth_center_x, mouth_center_y), 1, (255, 0, 255), 5)
      mouthxy = str(mouth_center_x) + " " +str(mouth_center_y)
      print (mouthxy)
    
      # cv2.polylines(cv_image, [inside_points],True, (0,255,255))
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(1)

    try:
      self.image_pub.publish(String(mouthxy))
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
