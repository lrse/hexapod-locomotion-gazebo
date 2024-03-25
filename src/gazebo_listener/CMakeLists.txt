cmake_minimum_required(VERSION 3.0.2)


project(gazebo_listener)

find_package(gazebo REQUIRED)


#add_compile_options(-std=c++11)

## Find ROS related packages/libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS ../../build
  LIBRARIES gazebo_listener
  CATKIN_DEPENDS roscpp)

## Build your executable
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(robot_gazebo_listener listener.cc)
target_link_libraries(robot_gazebo_listener ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} pthread)