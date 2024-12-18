# Arm Controller Package
Different modes of control for the end effector of the PX100 robotic arm.

# Launch Files

### px100_controller.launch.py
This launch file requires the modern robotics package to be installed using pip: `pip3 install modern-robotics`


Launch command for fake hardware:
```bash
ros2 launch arm_controller px100_controller.launch.py
```

Launch command for actual hardware:
```bash
ros2 launch arm_controller px100_controller.launch.py use_sim:=false
```

Launch command for admittance control demo
```bash
ros2 launch arm_controller px100_controller.launch.py use_admittance_control:=true
```

All parameters:
- threshold: Value from 0 to 1 defining joystick sensitivity; a larger number means the joystick will be less sensitive. Default is 0.75.

- use_rviz: Can be `true` or `false`. Launches RViz if set to `true`. Default is `true`.

- use_sim: Can be `true` or `false`. If `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's motion; if `false`, the real DYNAMIXEL driver node is run. This is used to run a simulation of the arm instead of using an actual arm. Default is `true`.

- use_admittance_control: launches the admittance controller node and the virutal pose publisher.
  - use_fake_force: launches an additional node to publish fake force measurements.

- publish_poses: launches the pose_publisher node

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
This package has a few nodes that can be used to publish data periodically. The current nodes are
- `pose_publisher`
- `ellipse_publisher`
- `force_publisher`
- `virtual_pose_publisher`

Each one has the following optional parameters that can be specified before running the node:
- `delay`: the amount of time to wait, in seconds, before starting to publish data.
- `frequency`: the time interval for publishing data. For example, if the frequency is 0.002, then the force publisher will publish a fake force reading every 0.002 seconds.
- `max_ticks`: the number of times data will be published before it is modified. For example, if max_ticks=500 was set for the force publisher, then after publishing a value 500 times, the value is incremented or set back to zero.

NOTE: delay and frequency need to be specified as floating point values. For example, use 1.0 instead of 1.

Example
```bash
ros2 run arm_controller pose_publisher --ros-args -r __ns:=/px100 -p delay:=0.0 -p frequency:=0.002 -p max_ticks:=250
```