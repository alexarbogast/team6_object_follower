#!/usr/bin/env python

############################################################
# go_to_goal node: moves the turtle bot to waypoints 
# while avoiding obstacles 
############################################################

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D

from pid import PID

import numpy as np
import math
from enum import Enum

# constants
OBST_THRESH = 0.4 # m
WAYPOINT_THRESH = 0.01 # m

BURGER_MAX_ANG_VEL = 2.84
BURGER_MAX_LIN_VEL = 0.2#0.22

class State(Enum):
	GoToGoal = 1
	AvoidObstacle = 2 

class Waypoint:
	def __init__(self, x=0, y=0):
		self.x, self.y = x, y

	def __str__(self):
		return "({0}, {1})".format(self.x, self.y)

	# get waypoint heading in radians
	def GetHeading(self):
		return np.arctan2(self.y, self.x)

	def GetDistance(self):
		return np.sqrt(self.x*self.x + self.y*self.y)

	def Scale(self, percentage):
		self.x += percentage*self.x
		self.y += percentage*self.y
		return self



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

class GoToGoal:
	def __init__(self, waypoints, waypoint_thresh=0.01):
		self._waypoints = waypoints
		self._thresh = waypoint_thresh

		self._ang_controller =  PID(1.7, 0, 0, BURGER_MAX_ANG_VEL, -BURGER_MAX_ANG_VEL)
		self._dist_controller = PID(0.8, 0, 0.1, BURGER_MAX_LIN_VEL, -BURGER_MAX_LIN_VEL)

	def GetCurrentWaypointRf(self, odom):
		waypoint = self._waypoints[0]
		waypoint_rf = Waypoint()

		x, y, theta  = odom.x, odom.y, odom.theta

		# world frame to robot frame rotation
		Mrot = np.matrix([[np.cos(theta), np.sin(theta)],
						 [-np.sin(theta), np.cos(theta)]])

		# vec from robot to waypoint in world frame
		rob_to_wp_x_wf = waypoint.x - x
		rob_to_wp_y_wf = waypoint.y - y

		# rotate to robot frame
		waypoint_rf.x = Mrot.item((0,0))*rob_to_wp_x_wf + Mrot.item((0,1))*rob_to_wp_y_wf
		waypoint_rf.y = Mrot.item((1,0))*rob_to_wp_x_wf+ Mrot.item((1,1))*rob_to_wp_y_wf

		return waypoint_rf

	def Control(self, odom, dt):
		ang_vel, lin_vel = 0, 0

		waypoint_rf = self.GetCurrentWaypointRf(odom)
		wp_heading = waypoint_rf.GetHeading()

		test_angle_thresh = 0.05

		if (waypoint_rf.GetDistance() < self._thresh):
			self._waypoints.pop(0)
			print('waypoint reached\n')

		#elif (wp_heading > test_angle_thresh):
		#	ang_vel = self._ang_controller.Calculate(0.01, wp_heading, 0)
		#	print('going for angle\n')

		else: # control position (robot frame)
			ang_vel = self._ang_controller.Calculate(dt, wp_heading, 0)
			lin_vel = self._dist_controller.Calculate(dt, waypoint_rf.x, 0)
			print('going for position\n')
			
		control_output = Twist()
		control_output.angular.z, control_output.linear.x = ang_vel, lin_vel

		# debug output
		print("---------------")
		print("Waypoint RF = {0}".format(waypoint_rf))
		print("Odom theta = {0}".format(np.degrees(odom.theta)))
		print("Heading = {0}\n".format(np.degrees(wp_heading)))

		print("Angular Vel = {0}".format(ang_vel))
		print("Linear Vel = {0}".format(lin_vel))

		return control_output

class AvoidObstacle:
	def __init__(self, obstacle_thresh=0.5):
		self._obstacle_thresh=obstacle_thresh
		self._ang_controller =  PID(1.7, 0, 0.1, BURGER_MAX_ANG_VEL, -BURGER_MAX_ANG_VEL)
		pass

	def GetMinDistLocation(self,lidar_data):
		filtered_lidar_data = []
		for point in lidar_data.ranges:
			point = 1000 if point == 0 else point
			filtered_lidar_data.append(point)

		angular_location_deg=np.argmin(filtered_lidar_data)
		angular_location=np.radians(angular_location_deg)
		return angular_location

	def Control(self,lidar_data,dt):
 		ang_loc=self.GetMinDistLocation(lidar_data)
		print(ang_loc)
		ang_loc = math.fmod(math.fmod(ang_loc, 2*np.pi) + 2*np.pi, 2*np.pi)
		if (ang_loc > np.pi):
			ang_loc -= 2*np.pi
		print(ang_loc)
		ang_vel = self._ang_controller.Calculate(dt, ang_loc, np.pi/2)
		if abs(ang_loc-np.pi/2)<0.2:
			lin_vel = 0.1
		else:
			lin_vel = 0
		control_output = Twist()
		control_output.angular.z, control_output.linear.x = ang_vel, lin_vel
		return control_output

class NavController:
	def __init__(self, waypoints,  obstacle_thresh=0.5, waypoint_thresh=0.01):
		self._obstacle_thresh = obstacle_thresh

		self._twist = Twist()
		self._time = rospy.get_time()

		self._odom = Odom()
		self._go_to_goal = GoToGoal(waypoints, waypoint_thresh)
		self._avoid_obstacle = AvoidObstacle(obstacle_thresh)

		self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.LidarCallback, queue_size=10)
		self._vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

		rospy.on_shutdown(self.ShutdownNav)

	def LidarCallback(self, lidar_data):
		state = self.FindState(lidar_data)

		# calculate dt
		time = rospy.get_time()
		dt = self._time - time
		dt = dt if dt > 0.001 else 0.001

		if state == State.GoToGoal:
			self._twist = self._go_to_goal.Control(self._odom.GetOdometry(), dt)
		else:
			self._twist = self._avoid_obstacle.Control(lidar_data,dt)

		self._vel_pub.publish(self._twist)
		self._time = rospy.get_time()

	def parse_distances(self, heading, lidar_data):
		EMPTY_VAL = 101
		LIDAR_RANGE_MIN = 0.12
		LIDAR_RANGE_MAX = 3.5
		HEADING_THRESH=5
		
		# do not parse for invalid heading
		if heading == EMPTY_VAL:
			return EMPTY_VAL

#		try:
		heading_range = np.array(range(int(heading) - HEADING_THRESH, 
									int(heading) + HEADING_THRESH + 1))

		## wrap angles
		lidar_ang = heading_range % 360		
		lidar_ang = (lidar_ang + 360) % 360 # force positive 

		lidar_ang = [ang-360 if ang > 180 else ang for ang in lidar_ang]

		dists = [lidar_data.ranges[i] for i in lidar_ang]
		filtered_dists=[]
		for point in dists:
			point = 1000 if point == 0 else point
			filtered_dists.append(point)

		min_dist = np.min(filtered_dists)

#			# filter values below lidar threshold
#			if min_dist < LIDAR_RANGE_MIN or min_dist > LIDAR_RANGE_MAX: 
#				min_dist = EMPTY_VAL
#		except IndexError:
#			min_dist =  EMPTY_VAL

		return min_dist

	def FindState(self, lidar_data):
		# TODO: parse lidar data to find if an obstacle
		# obstructs path to goal
		waypoint_rf = self._go_to_goal.GetCurrentWaypointRf(self._odom.GetOdometry())

		# find heading of current waypoint in robot frame
		heading = np.floor(np.degrees(waypoint_rf.GetHeading()))
		distance = waypoint_rf.GetDistance()

		min_lidar_dist=self.parse_distances(heading,lidar_data)

		state = State.GoToGoal
		if min_lidar_dist < self._obstacle_thresh:
			state = State.AvoidObstacle

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

	# scale for dead_reckoning
	for point in waypoints:
		point.Scale(0.10)

	state_controller = NavController(waypoints, OBST_THRESH, WAYPOINT_THRESH)

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
