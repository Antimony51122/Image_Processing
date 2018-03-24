import rospy
from sensor_msgs.msg import Joy

def callback(data):
	print data

def listener():
	rospy.init_node('node_1')
	rospy.Subscriber("/vive_left", Joy, callback)
	rospy.spin()
	print 1

listener()
