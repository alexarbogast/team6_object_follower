cmake_minimum_required(VERSION 3.0.2)
project(team6_object_follower)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	sensor_msgs
	cv_bridge
	roscpp
	rospy
)

catkin_python_setup()

catkin_package(
	 CATKIN_DEPENDS geometry_msgs sensor_msgs cv_bridge rospy
)

## Build ##
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(rotate_robot src/rotate_robot.cpp)
target_link_libraries(rotate_robot ${catkin_LIBRARIES})

## Install ##
catkin_install_python(PROGRAMS
	scripts/rotate_robot.py
	scripts/find_object.py
	scripts/param_assistant.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
