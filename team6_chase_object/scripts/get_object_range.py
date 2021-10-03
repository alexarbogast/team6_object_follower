#!/usr/bin/env python

############################################################
# get_object_range:
############################################################

import rospy

from sensor_msgs.msg import LaserScan

class ObjectTracker:
	def __init__(self):
		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1)

	def callback(self, data):
		rospy.loginfo(data)

if __name__=='__main__':
	rospy.init_node('object_range', anonymous = True)
	try:
		ot = ObjectTracker()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass