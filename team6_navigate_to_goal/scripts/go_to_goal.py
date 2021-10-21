#!/usr/bin/env python

############################################################
# go_to_goal node: moves the turtle bot to waypoints 
# while avoiding obstacles 
############################################################

import rospy
from navigate_to_goal import Waypoint, NavController

# constants
OBST_THRESH = 0.4 # m
WAYPOINT_THRESH = 0.01 # m

BURGER_MAX_ANG_VEL = 2.84
BURGER_MAX_LIN_VEL = 0.2#0.22

if __name__=='__main__':
	rospy.init_node('go_to_goal', anonymous=False)

	# velocity bounds
	ang_vel_bounds = [-BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL]
	lin_vel_bounds = [-BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL]

	# create waypoints
	point1 = Waypoint(1.5, 0)
	point2 = Waypoint(1.5, 1.4)
	point3 = Waypoint(0, 1.4)

	waypoints = [point1, point2, point3]

	# scale for dead_reckoning
	for point in waypoints:
		point.Scale(0.10)

	state_controller = NavController(waypoints, ang_vel_bounds, lin_vel_bounds, OBST_THRESH, WAYPOINT_THRESH)

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
