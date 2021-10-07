#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Twist, Pose2D

###################################
## VARIABLE DECLARATION AND SETUP
###################################

# System Parameters
location_topic='/target_loc'
BURGER_MAX_ANG_VEL = 2.84
BURGER_MAX_LIN_VEL = 0.22
EMPTY_VAL = 101
distance_setpoint = 0.8
max_distance_error = 3.5
angle_setpoint = 0
max_angle_error = 31

#PID Variables - Angular
kpA = 0#1.75	#Gains for PID controller
kiA = 0#0
kdA = 0#0.00
kA = [kpA, kiA, kdA]

integrated_errorA = 0 # Initialize some variables for PID controller
old_errorA = 0.0

#PID Variables - Linear
kpL = 1	#Gains for PID controller
kiL = 0
kdL = 0
kL = [kpL, kiL, kdL]

integrated_errorL = 0 # Initialize some variables for PID controller
old_errorL = 0.0

twist = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


###################################
## Function Declaration
###################################

def PID_method(error,delta_t,k,old_error,integrated_error):
	global old_time
	kp=k[0]
	ki=k[1] 
	kd=k[2]
	integrated_error = integrated_error+delta_t*error
	derivative_error = kd*(error-old_error)/delta_t
	proportional_error = kp*error
	old_error=error
	return integrated_error+derivative_error+proportional_error, old_error, integrated_error

def BoundOutput(output, abs_max):
	if output > abs_max:
		output = abs_max
	elif output < -abs_max:
		output = -abs_max
	return output 

def callback(data):
	global old_time, integrated_errorA, integrated_errorL, old_errorA, old_errorL, kA, kL
	angle = data.theta
	distance = data.x

	# Get the new time and compute the delta_t 
	new_time = rospy.get_time()
	delta_t = new_time - old_time
	if delta_t < 0.001:
		delta_t = 0.001
	
	# EMPTY_VAL indicates object has left the camera frame and no motion will occur
	# Otherwise, we use the PID to generate velocity commands from the distance error
	if distance == EMPTY_VAL or angle == EMPTY_VAL:
		twist.linear.x, twist.angular.z = 0.0, 0.0
	else:
		# Angular velocity control
		angle_error = angle - angle_setpoint
		angle_velocity, old_errorA, integrated_errorA = PID_method(angle_error/max_angle_error,
																   delta_t, kA, old_errorA,
																   integrated_errorA)

		angle_velocity = BoundOutput(angle_velocity, BURGER_MAX_ANG_VEL)
		angle_velocity = 0

		# Linear velocity control
		distance_error = distance - distance_setpoint
		linear_velocity, old_errorL, integrated_errorL = PID_method(distance_error,
																	delta_t, kL, old_errorL,
																	integrated_errorL) 

		rospy.loginfo(linear_velocity)
		linear_velocity = BoundOutput(linear_velocity, BURGER_MAX_LIN_VEL)

		twist.linear.x = linear_velocity
		twist.angular.z = angle_velocity

		
	#rospy.loginfo(twist)
	old_time=new_time
	pub.publish(twist)


def shutdown_hook():
	twist.angular.z = 0.0
	twist.linear.x = 0.0
	pub.publish(twist)

def Init():
	global old_time
	rospy.init_node('move_robot', anonymous=True)
	old_time = rospy.get_time()
	sub = rospy.Subscriber(location_topic, Pose2D, callback, queue_size=10)
	
if __name__=='__main__':
	try:
		Init()
		rospy.on_shutdown(shutdown_hook)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass