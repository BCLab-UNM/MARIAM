#!/bin/bash

TIME=$(date +%m-%d-%Y__%H:%M:%S)

ros2 bag record /ross/tf /ross/force /ross/odometry/filtered /ross/amcl_pose /monica/tf /monica/force /monica/odometry/filtered /monica/amcl_pose -o $PWD/src/mariam_experiments/bags/experiment_${TIME} &
ros2 run mariam_experiments run_experiment.py