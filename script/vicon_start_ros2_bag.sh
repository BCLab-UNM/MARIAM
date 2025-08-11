#!/bin/bash

ros2 bag record -o $PWD/$1  /tf /world_ross_pose /world_monica_pose \
 /ross/force /monica/force \
 /ross/admittance_control/stiffness /monica/admittance_control/stiffness \
 /ross/px100_virtual_pose /monica/px100_virtual_pose \
 /ross/cmd_vel /monica/cmd_vel
 

ros2 bag info $PWD/$1
