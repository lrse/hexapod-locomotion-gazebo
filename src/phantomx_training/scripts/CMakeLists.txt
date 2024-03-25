cmake_minimum_required(VERSION 3.5)

project(PhantomxRobotEnv)

# Set C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp 
                                        std_msgs genmsg 
                                        tf2 
                                        sensor_msgs)

add_library(loguru STATIC loguru/loguru.cpp)

add_executable(server env.cpp 
               joint_states.cpp 
               physics_contact_forces.cpp
               server.cpp)

target_link_libraries(server ${catkin_LIBRARIES} loguru dl)
include_directories(${catkin_INCLUDE_DIRS})