cmake_minimum_required(VERSION 2.8.3)
project(marvelmind_republisher)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs marvelmind_nav)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs marvelmind_nav)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(marvelmind_republisher src/marvelmind_republisher.cpp)
target_link_libraries(marvelmind_republisher ${catkin_LIBRARIES})