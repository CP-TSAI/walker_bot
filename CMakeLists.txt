cmake_minimum_required(VERSION 2.8.3)
project(walker_bot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs 
	move_base_msgs
	sensor_msgs
)


## Declare a catkin package
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(walker src/main.cpp src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})
