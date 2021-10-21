import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

class Odom:
	def __init__(self):
		self._global_pose = Pose2D()
		self._init_pose = Pose2D()
		self._init = True

		self._odom_sub = rospy.Subscriber('/odom', Odometry, self.UpdateOdometry, queue_size=10)
		self._odom_pub = rospy.Publisher('/debug_odom', Pose2D, queue_size=10)

	def ResetOdom(self, Odom):
		position = Odom.pose.pose.position

		q = Odom.pose.pose.orientation
		orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

		self._init_pose.theta = orientation
		Mrot = np.matrix([[np.cos(self._init_pose.theta),
						   np.sin(self._init_pose.theta)],
						 [-np.sin(self._init_pose.theta),
						   np.cos(self._init_pose.theta)]])
		self._init_pose.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
		self._init_pose.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y

	def UpdateOdometry(self, Odom):
		position = Odom.pose.pose.position

		q = Odom.pose.pose.orientation
		orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

		if self._init:
			self.ResetOdom(Odom)
			self._init = False

		Mrot = np.matrix([[np.cos(self._init_pose.theta),
						   np.sin(self._init_pose.theta)],
						 [-np.sin(self._init_pose.theta),
						   np.cos(self._init_pose.theta)]])

		self._global_pose.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self._init_pose.x
		self._global_pose.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self._init_pose.y
		self._global_pose.theta = orientation - self._init_pose.theta

		self._odom_pub.publish(self._global_pose)

	def GetOdometry(self):
		return self._global_pose