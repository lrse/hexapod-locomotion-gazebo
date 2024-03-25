#!/bin/bash

# Kill server process
pkill -f "./server"

# Kill screen process
pkill rosmaster

# Kill rosmaster process
pkill gzserver

# Kill gzserver process
pkill screen

screen -wipe

