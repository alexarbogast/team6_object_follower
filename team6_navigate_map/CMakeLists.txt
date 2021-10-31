cmake_minimum_required(VERSION 3.0.2)
project(team6_navigate_map)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

## Build ##
include_directories(
	${catkin_INCLUDE_DIRS}
)