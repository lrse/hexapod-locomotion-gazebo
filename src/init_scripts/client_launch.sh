#!/bin/bash

# Check if an argument is provided
if [ "$#" -eq 0 ]; then
	    echo "Usage: $0 <argument>"
	        exit 1
fi

echo "$1"

# Set the client directory path
client_dir="../phantomx_training/scripts/"

# Change directory to the client directory
cd "$client_dir"

# Run the Python script with the provided argument
python3 main.py "$1"
