cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_BUILD_TYPE Debug) 

project(phantomx_controller)

find_package(catkin REQUIRED)

## Find ROS related packages/libraries
find_package(catkin REQUIRED COMPONENTS
					roscpp
					std_msgs
					sensor_msgs)

## Find other packages/libraries
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS ../../build
  LIBRARIES phantomx_controller
  CATKIN_DEPENDS roscpp)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(robot_controller_gazebo
						src/phantomx.cpp
		       			src/main.cpp 
		       			src/inverse_kinematics.cpp
		       			src/gait_planning.cpp)

target_link_libraries(robot_controller_gazebo Eigen3::Eigen ${catkin_LIBRARIES})