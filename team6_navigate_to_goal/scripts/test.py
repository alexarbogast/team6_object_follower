#!/usr/bin/env python
import rospy
import time
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from go_to_goal import Odom

from pid import PID
from pid import Autotune

# global data
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

def odom_callback(data):
	global odom_data
	odom_data = data

def angular_output_func(output):
	vel = Twist()

	output = PID.Saturate(output, BURGER_MAX_ANG_VEL, -BURGER_MAX_ANG_VEL)

	vel.angular.z = output
	vel_pub.publish(vel)

def angular_feedback_func():
	return odom.GetOdometry().theta

def linear_output_func(output):
	vel = Twist()
	
	output = PID.Saturate(output, BURGER_MAX_LIN_VEL, -BURGER_MAX_LIN_VEL)

	vel.linear.x = output
	vel_pub.publish(vel)

def linear_feedback_func():
	return odom.GetOdometry().x

def linear_test(setpoint):
	thresh = 0.001

	Kp = 2
	Ki = 0
	Kd = 1

	pid = PID(Kp, Ki, Kd, BURGER_MAX_LIN_VEL, -BURGER_MAX_LIN_VEL)

	loop_rate = 0.02
	prev_time = time.time()

	error = 1000
	while abs(setpoint - linear_feedback_func()) > thresh:
		dt = time.time() - prev_time
		time.sleep(max(0,loop_rate -  dt))
		dt = dt if dt > loop_rate else loop_rate 

		output = pid.Calculate(dt, setpoint, linear_feedback_func())
		print(output)
		linear_output_func(output)

	linear_output_func(0)

def angular_test(setpoint):
	thresh = 0.001

	Kp = 0.12
	Ki = 0.146
	Kd = .0247

	pid = PID(Kp, Ki, Kd, BURGER_MAX_ANG_VEL, -BURGER_MAX_ANG_VEL)

	loop_rate = 0.02
	prev_time = time.time()

	error = 1000
	while abs(setpoint - angular_feedback_func()) > thresh:
		dt = time.time() - prev_time
		time.sleep(max(0,loop_rate -  dt))
		dt = dt if dt > loop_rate else loop_rate 

		output = pid.Calculate(dt, setpoint, angular_feedback_func())
		print(output)
		angular_output_func(output)

	angular_output_func(0)
	

if __name__=='__main__':
	rospy.init_node("autotune", anonymous = False)
	vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
	odom = Odom()

	try:
		#linear_test(0.25)
		#angular_test(math.pi/4)
		autotune = Autotune(8, 200)
		autotune.Tune(math.pi/4, angular_output_func, angular_feedback_func)
	except rospy.ROSInterruptException:
		pass
	