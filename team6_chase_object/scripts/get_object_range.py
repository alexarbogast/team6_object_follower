#!/usr/bin/env python

############################################################
# get_object_range:
############################################################

import rospy

from sensor_msgs.msg import LaserScan

class ObjectTracker:
	def __init__(self):
		self.lidar_sub = rospy.Subscriber("/scan", LaserScan)
		self.heading_sub = rospy.Subscriber("/heading", Float32)

		ts = message_filters.TimeSynchronizer([lidar_sub, heading_sub], 10)
		ts.registerCallback(callback)

		point = Point
		self.target_loc_pub = rospy.Publisher("/target_loc", Point, queuesize=1)

	def callback(self, scan, heading):
		rospy.loginfo(data)


if __name__=='__main__':
	rospy.init_node('object_range', anonymous = True)
	try:
		ot = ObjectTracker()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass