#!/bin/bash

# Define the list of bags
bags=(*.bag)

# Loop through each bag
for bag in "${bags[@]}"; do
    # Create a directory with the bag's name
    bag_dir="${bag%.bag}" # Remove the '.bag' extension
    # Change directory to the bag directory
    cd "$bag_dir"
    python3 /home/carter/MARIAM/misc/dataAnalysis/createPlotsFromOdomCSV.py
    python3 /home/carter/MARIAM/misc/dataAnalysis/createPlotsFromOdomCSVDiff.py
    # Change back to the parent directory
    cd ..
done


