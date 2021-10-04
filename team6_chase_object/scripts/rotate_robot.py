#!/usr/bin/env python
#Rotate the robot to face an object.

import sys
import rospy
from geometry_msgs.msg import Twist, Point

###################################
## VARIABLE DECLARATION AND SETUP
###################################

# System Parameters
BURGER_MAX_ANG_VEL = 2.84
center_of_image_x = 200

#PID Variables
kp = 1.875	#Gains for PID controller
ki = 0
kd = 0.125

integrated_error = 0 # Initialize some variables for PID controller
old_error=0.0
old_time=0

twist = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

###################################
## Function Declaration
###################################

def PID_method(error,delta_t):
	global integrated_error, old_error, old_time
	integrated_error=integrated_error+delta_t*error
	derivative_error=kd*(error-old_error)/delta_t
	proportional_error=kp*error
	old_error=error
	return(integrated_error+derivative_error+proportional_error)

def callback(data):
	# Get the new time and compute the delta_t 
	new_time=rospy.get_time()
	delta_t=new_time-old_time
	if delta_t<0.001:
		delta_t=0.001
	
	# Data<0 indicates the tracked object has left the camera frame and no motion will occur
	# Otherwise, we use the PID to generate velocity commands from the pixel error
	if data.x<0:
		twist.angular.z=0.0
		target_error=0
	else:
		x=data.x; data.y; data.z		
		target_error=x-center_of_image_x #negative == turn left, positive == turn right
		vel_out=-PID_method(target_error/200,delta_t) #
		if vel_out>BURGER_MAX_ANG_VEL:
			vel_out=BURGER_MAX_ANG_VEL
		elif vel_out<-BURGER_MAX_ANG_VEL:
			vel_out=-BURGER_MAX_ANG_VEL
		twist.angular.z=vel_out
	pub.publish(twist)
	old_time=new_time
	rospy.loginfo("Object location is: [%d  %d %d]", data.x, data.y, data.z)
	ropty.loginfo("Output rotation is: %d", twist.angular.z)

def shutdown_hook():
	twist.angular.z=0.0
	pub.publish(twist)

def Init():
	rospy.init_node('rotate_robot', anonymous=True)
	old_time=rospy.get_time()
	sub = rospy.Subscriber('target_loc', Point, callback,queue_size=10)

###################################
## MAIN
###################################

if __name__=='__main__':
	Init()
	rospy.on_shutdown(shutdown_hook)
	rospy.spin()

#if __name__=='__main__':
#	rospy.init_node('rotate_robot', anonymous=True)
#	old_time=rospy.get_time()
#	sub = rospy.Subscriber('target_loc', Point, callback,queue_size=10)
#	rospy.on_shutdown(shutdown_hook)
#	rospy.spin()