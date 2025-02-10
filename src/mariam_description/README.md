# mariam_description
This ROS2 Humble package holds all of the model files describing the MARIAM agents.

## Types of Modelling Formats
The three main file formats are described in more detail below:
- SDF (Simulation Description Format): An XML format used to detail robots, environments, sensors, and physics for simulation. Originally designed specifically for Gazebo.
- URDF (Unified Robot Description Format): An XML format for defining a robot’s structure (links, joints, and inertial properties) for visualization and control in ROS.
- XACRO (XML Macros): A macro language that generates URDF files, allowing for variables, macros, and conditionals to simplify complex model descriptions.

## Gazebo Model Format
Gazebo prefers models to be in SDF. Which is a burden since RVIZ and the rest of ROS software uses URDF. Two compromises can be made in this situation:
1. Use URDF for ROS and SDF for Gazebo, and ensure that your URDF and SDF files are as similar as possible.
2. Use URDF is Gazebo, however you must include simlation specific tags for Gazebo to be able to read it. The modifications necessary for using a URDF in Gazebo can be found here [here](https://classic.gazebosim.org/tutorials/?tut=ros_urdf).

On top of requiring SDF format, Gazebo has particular requirements for being able to add models in through the Gazebo application. The folder `./gazebo_models/mariam_description` is an example of that, it includes an sdf file and a config file which Gazebo uses to load in to simulation. If this folder is part of environment variable `$GAZEBO_MODEL_PATH`, Gazebo will allow you to drop in models manually from the *insert* tab in the Gazebo application window. To get this feature to work, I suggest adding the following to your `~/.bashrc` file:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\
~/MARIAM/src/mariam_description/gazebo_models/
```

## Launch Files
- `mariam_rviz.launch.py`: Launches RVIZ visualization of the `mariam.urdf.xacro` model. With a GUI for sliding joints around.

## Current Issues
- The `./xacro_models/xacro/mariam.urdf.xacro` model can spawn into Gazebo and does have a controller that interacts with ROS. However the movement of the joints is strange. The wheels do not spin at the same rate as the model has trouble turning. This can be tested by running `ros2 launch mariam_gazebo mariam_gazebo_spawn_urdf.launch.py` and using the rqt_robot_sterring GUI to drive.

## Notes
- Inspiration for orgazning the model `mariam.urdf.xacro` and its complimentary file `mariam_gazebo.urdf.xacro` comes from an robotics educational content creator's (github here)[https://github.com/joshnewans/articubot_one/tree/0085689ee023baac604268cdf7a9ce85a0ed7bae/description] 