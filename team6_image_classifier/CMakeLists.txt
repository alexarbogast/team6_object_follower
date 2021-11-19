cmake_minimum_required(VERSION 3.0.2)
project(team6_image_classifier)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	rospy
	sensor_msgs
	message_generation
)

catkin_python_setup()

add_service_files(
	FILES
	Classify.srv
)

generate_messages(
        DEPENDENCIES
)

catkin_package(
	CATKIN_DEPENDS rospy geometry_msgs sensor_msgs
)

## Build ##
include_directories(
	${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)