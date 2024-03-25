#!/bin/bash

# Set the server build directory path
server_build_dir="../phantomx_training/scripts/"

# Step 1: Change directory to the server build directory
cd "$server_build_dir"

# Step 2: Check if the 'build' folder exists, and create it if not
if [ ! -d "build" ]; then
	    mkdir -p "build"
	        echo "Created folder: $server_build_dir/build"
fi

# Step 3: Change directory to the 'build' folder
cd "build"

# Step 4: Run CMake, make, and execute the server
cmake ..
make
./server

