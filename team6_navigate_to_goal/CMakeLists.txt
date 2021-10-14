cmake_minimum_required(VERSION 3.0.2)
project(team6_navigate_to_goal)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	geometry_msgs
	sensor_msgs
	roscpp
	rospy
)

catkin_package(
	CATKIN_DEPENDS cv_bridge geometry_msgs roscpp rospy sensor_msgs
)

## Build ##
include_directories(
	${catkin_INCLUDE_DIRS}
)

## Install ##
catkin_install_python(PROGRAMS
	scripts/go_to_goal.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)