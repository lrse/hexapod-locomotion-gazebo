#!/bin/bash

# Check if an argument is provided
if [ "$#" -eq 0 ]; then
	    echo "Usage: $0 <argument>"
	        exit 1
fi

# Create a new screen session named "gazebo_launch"
screen -dmS gazebo_launch

# Run commands inside the screen session
screen -S gazebo_launch -X stuff 'chmod +x launch_gazebo.sh && ./launch_gazebo.sh\n'

echo "Launching Gazebo..."

sleep 30

# Create a new screen session named "launch_server"
screen -dmS launch_server

# Run commands inside the screen session
screen -S launch_server -X stuff 'chmod +x server_launch.sh && ./server_launch.sh\n'

echo "Launching Server..."

sleep 5

# Create a new screen session named "launch_client"
screen -dmS launch_client

# Run commands inside the client session
screen -S launch_client -X stuff 'chmod +x client_launch.sh && ./client_launch.sh '"$1"'\n' 

echo "Launching Client..."
