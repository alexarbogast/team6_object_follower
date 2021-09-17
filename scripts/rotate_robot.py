#!/usr/bin/env python
import sys

import rospy
from geometry_msgs.msg import Twist

BURGER_MAX_ANG_VEL = 2.84

if __name__=='__main__':
	rospy.init_node('rotate_robot', anonymous=True)

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	twist = Twist()
	twist.angular.x = 0.0; twist.angular.y = 0.0; 

	try:
		while not rospy.is_shutdown():
			twist.angular.z = BURGER_MAX_ANG_VEL
			pub.publish(twist)

	except:
		print("Unexpected error:", sys.exc_info()[0])

	finally:
	    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
	    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
	    pub.publish(twist)