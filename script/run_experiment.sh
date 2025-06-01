#!/bin/bash

TIME=$(date +%m-%d-%Y__%H:%M:%S)

# start up the ros2 bag
# TODO: add the desired topics we want to listen to
ros2 bag record /ross/commands/joint_group -o $PWD/src/mariam_experiments/bags/experiment_${TIME} &
ros2 run mariam_experiments run_experiment.py