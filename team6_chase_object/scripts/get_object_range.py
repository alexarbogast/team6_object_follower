#!/usr/bin/env python

############################################################
# get_object_range:
############################################################

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

HEADING_THRESH = 1
EMPTY_VAL = 101

class ObjectTracker:
	def __init__(self):
		self.lidar_data = LaserScan()
		
		# subscribers
		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
		self.heading_sub = rospy.Subscriber("/heading", Float32, self.heading_callback, queue_size=1)
		
		# publishers
		self.target_pub = rospy.Publisher("/target_loc", Pose2D, queue_size=1)

	def lidar_callback(self, data):
		self.lidar_data = data

	def heading_callback(self, data):
		heading = data.data
		
		target_loc = Pose2D()
		
		target_loc.x = self.parse_distances(heading)
		target_loc.theta = heading

		self.target_pub.publish(target_loc)
		rospy.loginfo(target_loc)

	def parse_distances(self, heading):
		# do not parse for invalid heading
		if heading == EMPTY_VAL:
			return EMPTY_VAL

		try:
			heading_range = np.array(range(int(heading) - HEADING_THRESH, 
										   int(heading) + HEADING_THRESH + 1))

			## wrap angles
			lidar_ang = heading_range % 360		
			lidar_ang = (lidar_ang + 360) % 360 # force positive 

			lidar_ang = [ang-360 if ang > 180 else ang for ang in lidar_ang]

			dists = [self.lidar_data.ranges[i] for i in lidar_ang]
			med_dist = np.median(dists)
		except IndexError:
			return EMPTY_VAL

		return med_dist

if __name__=='__main__':
	rospy.init_node('object_range', anonymous = True)
	try:
		ot = ObjectTracker()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass