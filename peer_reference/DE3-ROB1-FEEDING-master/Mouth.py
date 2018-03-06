from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import message_filters
from message_filters import Subscriber
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imutils import face_utils
import numpy as np
import argparse
import imutils
import dlib
import cv2

#dont forget to $ roscore and  $ roslaunch openni2_launch openni2.launch
# this code outputs 3 topics: xyz of mouth (pixels) , mouth status and face status
# locate the predictors and detectors
predictor_path = "/home/robin/DE3-ROB1-FEEDING/perception/shape_predictor_68_face_landmarks.dat"
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)


class Mouth:

	def __init__(self):

		self.face_status_pub = rospy.Publisher("/face_status", String, queue_size=10)
		self.mouth_xyz_pub = rospy.Publisher("/mouth_xyz", Point, queue_size=10)
		self.mouth_status_pub = rospy.Publisher("/mouth_status", String, queue_size=10)

		self.bridge = CvBridge()
		image_sub = Subscriber("/camera/rgb/image_rect_color",Image)
		depth_sub = Subscriber("/camera/depth_registered/image_raw", Image)

		tss = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],queue_size=10, slop=0.5)															
		tss.registerCallback(self.callback)
		print('init')

	def callback(self,img,depth):

		try:
			depth_image_raw = self.bridge.imgmsg_to_cv2(depth, "passthrough")
			depth_image = ((255*depth_image_raw)).astype(np.uint8)
			
		except CvBridgeError as e:
			print(e)
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)

		#resizing image speeds it up! cloning to make changes without changing perception
		cv_image = imutils.resize(cv_image, width=500)
		depth_image = imutils.resize(depth_image, width=500)
		depth_image_clone = depth_image
		cv_image_clone = cv_image

		#converting the image to grayscale 
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		# detect faces
		rects = detector(gray, 1)
		# establish if any faces present 
		if rects != None:
			face_status = 'present'
		else: 
			face_status = 'absent'

		print(face_status)

		self.face_status_pub.publish(face_status)

		# #are picture frames different size? both are 480,640 
		# # print(depth_image_clone.shape)

		#for each face identify landmarks 
		for (i, rect) in enumerate(rects):
			shape = predictor(gray, rect)
			shape = face_utils.shape_to_np(shape)
			features = face_utils.FACIAL_LANDMARKS_IDXS.items()
			mouth = features[0]
			points = shape[mouth[1][0]:mouth[1][1]]
			# print mouth points:
			# for (x,y) in points:
			#   cv2.circle(cv_image, (x, y), 1, (0, 0, 255), -1)

			# inside_points = shape[60:68]
			mouth_top = shape[62]
			mouth_bottom = shape[66]
			mouth_left = shape[61]
			mouth_right = shape[65]

			mouth_center_x = mouth_bottom[0] +(mouth_top[0]-mouth_bottom[0])/2
			mouth_center_y = mouth_bottom[1] +(mouth_top[1]-mouth_bottom[1])/2
			# open close mouth ratio
			mouth_ratio = float(abs(mouth_top[1]- mouth_bottom[1])/(abs(mouth_left[0] - mouth_right[0])))

			if mouth_ratio > 0: 
				mouth_status = 'open'
			else: 
				mouth_status = 'closed'
		

			#cv2.circle(cv_image, (mouth_center_x, mouth_center_y), 1, (255, 0, 255), 5)

			# mouth center: 
			try:
				mouth_center_z = depth_image[mouth_center_x, mouth_center_y]
			except:
				mouth_center_z = 0
			mouthxyz = str(mouth_center_x) + " " +str(mouth_center_y) + " " +str(mouth_center_z)
			cv2.circle(depth_image_clone, (mouth_center_x, mouth_center_y), 1, (255, 0, 0), 5)

		 	print ("mouth %s x, y, z = " %(i) ,mouthxyz, 'status = ', mouth_status)
		 	xyz = Point()
			xyz.x = mouth_center_x
			xyz.y = mouth_center_y
			xyz.z = mouth_center_z
			self.mouth_xyz_pub.publish(xyz)
			
			self.mouth_status_pub.publish(mouth_status)

		# print(depth_image_clone[mouth_center_x, mouth_center_y])

		# print(depth_image[240,500])
		
		cv2.imshow("Depth window", depth_image_clone)
		cv2.imshow("Image window", cv_image_clone)
		cv2.waitKey(1)

		

		# try:
		#   self.image_pub.publish(String(mouthxyz))
		# except CvBridgeError as e:
		#   print(e)

		# print(np.size(cv_image) , np.size(depth_image))


def main(args):
		ic = Mouth()
		rospy.init_node('Mouth', anonymous=True)
		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	print("here")
	main(sys.argv)
