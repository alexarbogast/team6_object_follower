import numpy as np
import math

from geometry_msgs.msg import Twist

from pid import PID

class AvoidObstacle:
	def __init__(self, ang_vel_bounds, obstacle_thresh=0.5):
		self._obstacle_thresh=obstacle_thresh
		self._ang_controller =  PID(1.7, 0, 0.1, ang_vel_bounds[1], ang_vel_bounds[0])
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
			lin_vel = 0.05

		control_output = Twist()
		control_output.angular.z, control_output.linear.x = ang_vel, lin_vel
		return control_output