#!/usr/bin/env python

############################################################
# go_to_goal:
############################################################

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D

import numpy as np
from enum import Enum

# constants
OBST_THRESH = 0.2 #m

class State(Enum):
	GoToGoal = 1
	AvoidObstacle = 2 

class Waypoint:
	def __init__(self, x, y):
		self.X, self.Y = x, y

class Odom:
	def __init__(self):
		self._global_pose = Pose2D 
		self._init_pose = Pose2D
		self._init = True

		self._odom_sub = rospy.Subscriber('/odom', Odometry, self.UpdateOdometry, queue_size=10)

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
		self._global_pose.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
		self._global_pose.theta = orientation - self._init_pose.theta

	def GetOdometry(self):
		return self._global_pose

class GoToGoal:
	def __init__(self, waypoints):
		self._waypoints = waypoints

	def control(self):
		pass

class AvoidObstacle:
	def __init__(self):
		pass

	def control(self):
		pass

class NavController:
	def __init__(self, waypoints,  obstacle_thresh=0.1):
		self._state = State.GoToGoal
		self._obstacle_thresh = obstacle_thresh

		self._twist = Twist()
		self._odom = Odometry()

		self._odom = Odom()
		self._go_to_goal = GoToGoal(waypoints)
		self._avoid_obstacle = AvoidObstacle()

		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.LidarCallback, queue_size=1)
		self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

		rospy.on_shutdown(self.ShutdownNav)

	def LidarCallback(self, lidar_data):
		self.FindState(lidar_data)

		if self._state == State.GoToGoal:
			self._twist = self._go_to_goal.control()
		else:
			self._twist = self._avoid_obstacle.control()

		self._vel_pub.publish(self._twist)

	def FindState(self, lidar_data):
		# TODO: parse lidar data to find min dist
		min_lidar_dist = 0.3

		if min_lidar_dist < self._obstacle_thresh:
			self._state = State.AvoidObstacle
		else:
			self._state = State.GoToGoal
	
	# shutdown hook
	def ShutdownNav(self):
		self._twist.angular.x, self._twist.angular.y, self._twist.angular.z = 0.0, 0.0, 0.0
		self._twist.linear.x, self._twist.linear.y, self._twist.linear.z = 0.0, 0.0, 0.0 

		self._vel_pub.publish(self._twist)


if __name__=='__main__':
	rospy.init_node('go_to_goal', anonymous=False)

	# create waypoints
	point1 = Waypoint(1.5, 0)
	point2 = Waypoint(1.5, 1.4)
	point3 = Waypoint(0, 1.4)

	waypoints = [point1, point2, point3]

	state_controller = NavController(waypoints, OBST_THRESH)

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
