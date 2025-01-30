# Arm Controller Package
Different modes of control for the end effector of the PX100 robotic arm.

## Launch Files

### xsarm_joy.launch.py
This launch file requires the modern robotics package to be installed using pip: `pip3 install modern-robotics`
Documentation for controller input: https://wiki.ros.org/joy#Microsoft_Xbox_360_Wired_Controller_for_Linux

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

Launch command (with fake hardware):
```bash
ros2 launch arm_controller xsarm_joy.launch.py
```

Launch command (with actual hardware):
```bash
ros2 launch arm_controller xsarm_joy.launch.py use_sim:=false
```

Launch command (with admittance control demo)
```bash
ros2 launch arm_controller xsarm_joy.launch.py use_admittance_control:=true
```

Parameters:
- threshold: Value from 0 to 1 defining joystick sensitivity; a larger number means the joystick will be less sensitive. Default is 0.75.

- use_rviz: Can be `true` or `false`. Launches RViz if set to `true`. Default is `true`.

- use_sim: Can be `true` or `false`. If `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's motion; if `false`, the real DYNAMIXEL driver node is run. This is used to run a simulation of the arm instead of using an actual arm. Default is `true`.

- use_admittance_control: launches three nodes to run a demo of admittance control.

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

## Publisher nodes
This package has a few nodes that can be ran to publish data periodically. The current nodes are
- `high_freq_pose_publisher`
- `ellipse_publisher`
- `force_publisher`
- `virtual_pose_publisher`

Each one has the following optional parameters that can be specified before running the node:
- delay: the amount of time in seconds to wait before publishing poses.
- frequency: the amount of time in seconds to wait before publishing another pose.
- max_ticks: the number of ticks before switching to a different pose, point, etc.

NOTE: delay and frequency need to be specified as floating point values. For example, use 1.0 instead of 1.

Command example
```bash
ros2 run arm_controller high_freq_pose_publisher --ros-args -p delay:=0.0 -p frequency:=0.002 -p max_ticks:=250
```