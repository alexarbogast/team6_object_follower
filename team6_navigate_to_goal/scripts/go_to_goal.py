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
OBST_THRESH = 0.2 # m
WAYPOINT_THRESH = 0.01 # m

class State(Enum):
	GoToGoal = 1
	AvoidObstacle = 2 

class Waypoint:
	def __init__(self, x=0, y=0):
		self.x, self.y = x, y

	def __str__(self):
		return "({0}, {1})".format(self.x, self.y)

	# get waypoint heading in degrees
	def GetHeading(self):
		return np.degrees(np.arctan2(self.y, self.x))

	def GetDistance(self):
		return np.sqrt(self.x*self.x + self.y*self.y)

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
	def __init__(self, waypoints, waypoint_thresh=0.01):
		self._waypoints = waypoints
		self._thresh = waypoint_thresh

	# get the current waypoint in the robot frame
	def GetCurrentWaypointRf(self, odom):
		waypoint = self._waypoints[0]
		waypoint_rf = Waypoint()

		x, y, theta  = odom.x, odom.y, odom.theta
		Mrot = np.matrix([[np.cos(theta), -np.sin(theta)],
						  [np.sin(theta),  np.cos(theta)]])

		waypoint_rf.x, waypoint_rf.y = waypoint.x - x, waypoint_rf.y - y

		# rotate waypoint
		waypoint_rf.x = Mrot.item((0,0))*waypoint_rf.x + Mrot.item((0,1))*waypoint_rf.y
		waypoint_rf.y = Mrot.item((1,0))*waypoint_rf.x + Mrot.item((1,1))*waypoint_rf.y

		return waypoint_rf

	def Control(self, odom):
		return Twist()

class AvoidObstacle:
	def __init__(self):
		pass

	def Control(self):
		return Twist()

class NavController:
	def __init__(self, waypoints,  obstacle_thresh=0.1, waypoint_thresh=0.01):
		self._obstacle_thresh = obstacle_thresh

		self._twist = Twist()

		self._odom = Odom()
		self._go_to_goal = GoToGoal(waypoints, waypoint_thresh)
		self._avoid_obstacle = AvoidObstacle()

		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.LidarCallback, queue_size=10)
		self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

		rospy.on_shutdown(self.ShutdownNav)

	def LidarCallback(self, lidar_data):
		state = self.FindState(lidar_data)

		if state == State.GoToGoal:
			self._twist = self._go_to_goal.Control(self._odom.GetOdometry())
		else:
			self._twist = self._avoid_obstacle.Control()

		self._vel_pub.publish(self._twist)

	def FindState(self, lidar_data):
		# TODO: parse lidar data to find if an obstacle
		# obstructs path to goal
		
		waypoint_rf = self._go_to_goal.GetCurrentWaypointRf(self._odom.GetOdometry())

		# find heading of current waypoint in robot frame
		heading = waypoint_rf.GetHeading()
		distance = waypoint_rf.GetDistance()

		min_lidar_dist = 0.3

		state = State.GoToGoal
		if min_lidar_dist < self._obstacle_thresh:
			state = State.AvoidObstacle
		else:
			state = State.GoToGoal

		return state

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

	state_controller = NavController(waypoints, OBST_THRESH, WAYPOINT_THRESH)

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
