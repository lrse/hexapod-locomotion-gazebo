#!/bin/bash

# Set the directory paths
heightmap_plugin_dir="../phantomx_heightmap_plugin"
gazebo_listener_dir="../gazebo_listener"
project_dir="/home/tania/Codigos/phantomx"

# Install and configure phantomx_heightmap_plugin
cd "$heightmap_plugin_dir"
./install.sh
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)/build

# Install gazebo_listener
cd "$gazebo_listener_dir"
./install.sh

# Launch robot in Gazebo
cd "$project_dir"
catkin_make
source devel/setup.bash
roslaunch phantomx run_robot_gazebo.launch
