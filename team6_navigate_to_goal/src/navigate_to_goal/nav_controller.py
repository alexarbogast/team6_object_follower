import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from utilities import *
from odom import Odom
from go_to_goal import GoToGoal
from avoid_obstacle import AvoidObstacle

class NavController:
	def __init__(self, waypoints, ang_vel_bounds, lin_vel_bounds, obstacle_thresh=0.5, waypoint_thresh=0.01):
		self._obstacle_thresh = obstacle_thresh

		self._twist = Twist()
		self._time = rospy.get_time()

		self._odom = Odom()
		self._go_to_goal = GoToGoal(waypoints, ang_vel_bounds, lin_vel_bounds, waypoint_thresh)
		self._avoid_obstacle = AvoidObstacle(ang_vel_bounds, obstacle_thresh)

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
			(self._twist, pause) = self._go_to_goal.Control(self._odom.GetOdometry(), dt)
		else:
			self._twist = self._avoid_obstacle.Control(lidar_data,dt)

		self._vel_pub.publish(self._twist)
		
		if pause:
			rospy.sleep(5)

		self._time = rospy.get_time()

	def parse_distances(self, heading, lidar_data):
		HEADING_THRESH=30
	
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