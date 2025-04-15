#!/bin/bash

#### Step 1
# launch the px100 controller without admittance controller
# this moves the arms into the first position
read -p "Press enter to apply pressure on the object "

ros2 run arm_controller pose_updater squeeze --ros-args --log-level DEBUG

read -p "Press enter to lift the object "

ros2 run arm_controller pose_updater lift --ros-args --log-level DEBUG