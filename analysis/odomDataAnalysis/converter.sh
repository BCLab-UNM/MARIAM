#!/bin/bash

# Define the list of bags
bags=(*.bag)

# Loop through each bag
for bag in "${bags[@]}"; do
    # Create a directory with the bag's name
    bag_dir="${bag%.bag}" # Remove the '.bag' extension
    mkdir -p "$bag_dir"
    
    # Change directory to the bag directory
    cd "$bag_dir"

    # Extract desired ROS topics to CSV files
    rostopic echo -b "../$bag" -p /r2/odom > r2_odom.csv
    rostopic echo -b "../$bag" -p /r4/odom > r4_odom.csv
    rostopic echo -b "../$bag" -p /r2/bridge/debugPID > r2_pid.csv
    rostopic echo -b "../$bag" -p /r4/bridge/debugPID > r4_pid.csv

    # Change back to the parent directory
    cd ..
done
