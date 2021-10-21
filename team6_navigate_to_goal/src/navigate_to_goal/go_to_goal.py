import numpy as np

from geometry_msgs.msg import Twist, Pose2D

from utilities import *
from pid import PID

class GoToGoal:
	def __init__(self, waypoints, ang_vel_bounds, lin_vel_bounds, waypoint_thresh=0.01):
		self._waypoints = waypoints
		self._thresh = waypoint_thresh

		self._ang_controller =  PID(1.7, 0, 0, ang_vel_bounds[1], ang_vel_bounds[0])
		self._dist_controller = PID(0.8, 0, 0.1, lin_vel_bounds[1], lin_vel_bounds[0])

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