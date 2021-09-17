#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist, Point

BURGER_MAX_ANG_VEL = 2.84
center_of_image_x=200

twist = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def rot_vel(error):
	max_v=BURGER_MAX_ANG_VEL
	max_error_magnitude=center_of_image_x
	vel=-(max_v/max_error_magnitude)*error	#linear interpolation
	return vel

def callback(data):
	if data is None:
		twist.angular.z=0.0

	else:
		x=data.x; data.y; data.z		#need to double check that x is horizontal
		target_error=x-center_of_image_x #negative == turn left, positive == turn right
		vel_out=rot_vel(target_error)
		twist.angular.z=vel_out

	pub.publish(twist)	
	print(data)	

if __name__=='__main__':
	rospy.init_node('rotate_robot', anonymous=True)
	sub = rospy.Subscriber('target_loc', Point, callback,queue_size=10)
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