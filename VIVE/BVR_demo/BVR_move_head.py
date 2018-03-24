import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import tf
import numpy as np

rospy.init_node("move_baxter_head")
head = baxter_interface.Head()
listener = tf.TransformListener()
axis = 1

done = False
while not done:
	try:
		_, quat = listener.lookupTransform('/hmd', '/world_vive', rospy.Time(0))
		done = True
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue
angle0 = np.array(tf.transformations.euler_from_quaternion(quat))[axis]

rate = rospy.Rate(0.1)
while not rospy.is_shutdown():
	try:
		_, quat = listener.lookupTransform('/hmd', '/world_vive', rospy.Time(0))
		angle = np.array(tf.transformations.euler_from_quaternion(quat))[axis]
		diff = -angle
		threshold = 1.0
		if diff < -threshold: diff = -threshold
		elif diff > threshold: diff = threshold
		head.set_pan(diff, speed=1.0, timeout=0)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue
