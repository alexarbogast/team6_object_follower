#!/usr/bin/env python
import rospy

if __name__=='__main__':
    rospy.init_node('go_to_goal', anonymous=False)
    rospy.logwarn("go_to_goal")