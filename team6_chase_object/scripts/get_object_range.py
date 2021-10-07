#!/usr/bin/env python

############################################################
# get_object_range:
############################################################

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

HEADING_THRESH = 10

class ObjectTracker:
	def __init__(self):
		# subscribers
		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
		self.heading_sub = rospy.Subscriber("/heading", Float32, self.heading_callback, queue_size=1)
		
		# publishers
		self.target_pub = rospy.Publisher("/target_loc", Pose2D, queue_size=1)

		self.lidar_data = LaserScan()

	def lidar_callback(self, data):
		self.lidar_data = data

	def heading_callback(self, data):
		heading = data.data
		
		target_loc = Pose2D()
		target_loc.x, target_loc.theta = 1, 30

		self.target_pub.publish(target_loc)
		self.parse_distances(heading)

	def parse_distances(self, heading):
		angle_step=1
		if heading>0:
			range_index=heading/angle_step
		elif heading<0:
			range_index=(360+heading)/angle_step

		range_index = int(range_index)

		range_set =[]
		for i in range(-2, 3):
			range_set[i]=self.lidar_data.ranges[range_index + i]
		
		object_distance=np.median(range_set)
		rospy.loginfo(object_distance)

		#i = 0
		#self.lidar_data.ranges[]
		#for range in self.lidar_data.ranges:
		#	
		#	i += 1
		
if __name__=='__main__':
	rospy.init_node('object_range', anonymous = True)
	try:
		ot = ObjectTracker()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass