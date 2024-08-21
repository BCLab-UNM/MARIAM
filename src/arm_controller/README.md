# Arm Controller Package
Different modes of control for the end effector of the PX100 robotic arm.

## Table of commands for xsarm_joy.launch.py
This launch file requires the modern robotics package be installed using pip: `pip3 install modern-robotics`
Documentation: https://wiki.ros.org/joy#Microsoft_Xbox_360_Wired_Controller_for_Linux

Note: the buttons on the controller fall under two categories: buttons and axes. The following table documents which category each button falls under and the action it will perform when pressed.

**Buttons**
| Value | Button       | Action                   |
|-------|--------------|--------------------------|
|   0   | A            |                          |
|   1   | B            |                          |
|   2   | X            | Go to lift pose      |
|   3   | Y            | Go to starting lift pose |
|   4   | LB           | Rotate waist counterclockwise |
|   5   | RB           | Rotate waist clockwise   |
|   6   | back         | Go to sleep pose         |
|   7   | start        | Go to home pose          |
|   8   | power        | Enable torque            |
|   9   | LS (pressed) |  |
|  10   | RS (pressed) |    |

**Axes**
| Value | Button                 | Action                  |
|-------|------------------------|-------------------------|
|   0   | LS (left/right)        | Move end effector along X axis |
|   1   | LS (up/down)           | Move end effector along Z axis |
|   2   | LT                     |                         |
|   3   | RS (left/right)        |                         |
|   4   | RS (up/down)           | Move end effector pitch |
|   5   | RT                     |                         |
|   6   | Cross key (left/right) | Change speed type       |
|   7   | Cross key (up/down)    | Change speed            |


## Experiment node arguments
The experiment node has four arguments that can be specified before running the node.
Parameters:
- delay: the amount of time in seconds to wait before publishing poses.
- duration: the amount of time in seconds to publish poses for.
- frequency: the amount of time in seconds to wait before publishing another pose.
- max_ticks: the number of ticks before switching to a different pose, point, etc.
- exp_type: sets the type of experiment you want to perform.
  - `exp_type:=constraint-exp` will start publishing poses to the topic "/joy_target_pose" and was used for testing constrained path planning with moveit2. Does not use the max_ticks parameter.
  - `exp_type:=ellipse-trace` will start publishing waypoints along an ellipse to the "/high_freq_publisher" topic. max_ticks will affect the amount of time that will pass before publishing the next waypoint.
  - `exp_type:=force-exp` will start publishing floating point values to the topic "/force". When `max_ticks` is reached, the value will get incremented by a random value, or get reset back to zero if it's greater than or equal to 2.
  - If a value is not provided, then three poses will start getting published to "/high_freq_publisher".

NOTE: delay, duration, and frequency need to be specified as floating point values (for example: use 1.0 instead of 1).

Example command: `ros2 run arm_controller experiment --ros-args -p delay:=1.0 duration:=150.0 frequency:=0.002 max_ticks:=200 --exp_type:=ellipse-trace`

## Launch Files

### xsarm_joy.launch.py
Launch command (with fake hardware):
```bash
ros2 launch arm_controller xsarm_joy.launch.py
```

Launch command (with actual hardware):
```bash
ros2 launch arm_controller xsarm_joy.launch.py use_sim:=false
```

Additional parameters:
- threshold: Value from 0 to 1 defining joystick sensitivity; a larger number means the joystick will be less sensitive. Default is 0.75.

- use_rviz: Can be `true` or `false`. Launches RViz if set to `true`. Default is `true`.

- use_sim: Can be `true` or `false`. If `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's motion; if `false`, the real DYNAMIXEL driver node is run. This is used to run a simulation of the arm instead of using an actual arm. Default is `true`.

Best practices when moving the arm using the Python-ROS interface: https://docs.trossenrobotics.com/interbotix_xsarms_docs/python_ros_interface.html#tips-best-practices

### moveit_joy_constrained.launch.py
Launch with:

```bash
ros2 launch arm_controller moveit_joy_constrained.launch.py
```

Or:
```bash
ros2 launch arm_controller moveit_joy_constrained.launch.py hardware_type:=actual
```

### moveit_joy.launch.py
Launch with:

```bash
ros2 launch arm_controller moveit_joy.launch.py \
  robot_model:=px100 \
  hardware_type:=fake
  use_joy:=true
```

### xsarm_moveit_listener.launch.py
Launch with:

```bash
ros2 launch arm_controller moveit_listener.launch.py \
  robot_model:=px100 \
  hardware_type:=fake
```

### Parameters
Each launch file will use some or all of these parameters, here are their options.
- hardware_type: actual, gz_classic, fake
- robot_model: px100
- controller:= xbox360
- use_rviz:= true, false
- use_joy:= true, false
