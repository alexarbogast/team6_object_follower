cmake_minimum_required(VERSION 3.0.2)
project(team6_navigate_map)

find_package(catkin REQUIRED COMPONENTS geometry_msgs)

catkin_package(
	CATKIN_DEPENDS rospy geometry_msgs
)

## Build ##
include_directories(
	${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
	scripts/navigate_map
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)