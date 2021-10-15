cmake_minimum_required(VERSION 3.0.2)
project(team6_chase_object)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	geometry_msgs
	sensor_msgs
	roscpp
	rospy
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS cv_bridge geometry_msgs roscpp rospy sensor_msgs
)

## Build ##
include_directories(
	${catkin_INCLUDE_DIRS}
)

## Install ##
catkin_install_python(PROGRAMS
	scripts/detect_object.py
	scripts/get_object_range.py
	scripts/chase_object.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)