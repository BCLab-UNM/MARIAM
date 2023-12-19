# MARIAM (Multi-Agent Robust & Intelligent Autonomous Manipulation)

## Submodele Setup
Insert px100 rotation on line 64 of  `/MARIAM/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/px100.urdf.xacro` directly after `<child/>` statement
```xml
<origin
    rpy="-${pi/2} 0 -${pi/2}"
    xyz="0.0 0.0 0.06"/> 
``
### Experiment 1 Setup
No arguements are needed the defaults are already set.
On laptop: `ros2 launch arm_controller laptop_demo1.launch.py`
On Monica: `ros2 launch arm_controller monica_demo1.launch.py`
On Ross: `ros2 launch arm_controller ross_demo1.launch.py`

Perform pickup of box.
Then on laptop launch  `<move base node>`