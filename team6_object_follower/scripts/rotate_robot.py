#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist, Point
#import time

integrated_error = 0
old_error=0.0
old_time=0
BURGER_MAX_ANG_VEL = 2.84
center_of_image_x = 200

kp=2
ki=0
kd=0.2

twist = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def PID_method(error,delta_t):
	#get delta t
	global integrated_error, old_error, old_time
	integrated_error=integrated_error+delta_t*error
	derivative_error=kd*(error-old_error)/delta_t
	proportional_error=kp*error
	old_error=error
	old_time=rospy.get_time()
	return(integrated_error+derivative_error+proportional_error)

def rot_vel(error):
	max_v=BURGER_MAX_ANG_VEL
	max_error_magnitude=center_of_image_x
	vel= -(0.75*max_v/max_error_magnitude)*error	#linear interpolation
	return vel

def callback(data):
	new_time=rospy.get_time()#get new time, get delta t
	delta_t=new_time-old_time
	if delta_t<0.001:
		delta_t=0.001
	#compute the error
	
	if data.x<0:
		twist.angular.z=0.0
		target_error=0
	else:
		x=data.x; data.y; data.z		#need to double check that x is horizontal
		target_error=x-center_of_image_x #negative == turn left, positive == turn right
		#vel_out=rot_vel(target_error)
		vel_out=-PID_method(target_error/200,delta_t)
		if vel_out>BURGER_MAX_ANG_VEL:
			vel_out=BURGER_MAX_ANG_VEL
		elif vel_out<-BURGER_MAX_ANG_VEL:
			vel_out=-BURGER_MAX_ANG_VEL
		twist.angular.z=vel_out
	pub.publish(twist)	
	print(data)	

def shutdown_hook():
	twist.angular.z=0.0
	pub.publish(twist)

if __name__=='__main__':
	rospy.init_node('rotate_robot', anonymous=True)
	old_time=rospy.get_time()
	sub = rospy.Subscriber('target_loc', Point, callback,queue_size=10)
	rospy.on_shutdown(shutdown_hook)
	rospy.spin()
	

	#try:
	#	while not rospy.is_shutdown():
	#		#twist.angular.z = BURGER_MAX_ANG_VEL
	#		#twist.angular.z=vel_out
#
	#except:
	#	print("Unexpected error:", sys.exc_info()[0])
#
	#finally:
	#    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
	#    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
	#    pub.publish(twist)