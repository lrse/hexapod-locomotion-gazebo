#!/bin/bash

# Check if an argument is provided
if [ "$#" -eq 0 ]; then
	    echo "Usage: $0 <argument>"
	        exit 1
fi

echo "$1"
