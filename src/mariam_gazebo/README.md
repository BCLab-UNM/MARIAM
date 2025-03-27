# mariam_gazebo
This ROS2 Humble package holds all of the model files describing the MARIAM agents.

## Launch Files
- `mariam_gazebo_spawn_urdf.launch.py`: Uses the model `mariam.urdf.xacro` to start the robot state publisher and spawn the same model into Gazebo. This currently works but has some bugs descibed below.
- `mariam_gazebo_spawn_sdf.launch.py`: Uses the model `mariam.urdf.xacro` to start the robot state publisher. It then uses the model `mariam.sdf` to spawn in Gazebo. This is also experiencing a similar bug.
- `mariam_gazebo_navstack_map.launch.py`: Uses an SDF model for Gazebo. Demonstrates navigation using a a map as input.
- `mariam_gazebo_navstack_slam.launch.py`: Uses an SDF model for Gazebo. Demonstrates SLAM.
- `basic_mobile_robot_gazebo_spawn.launch.py`: This is from a tutorial on robot navigation in Gazebo. The full turorial can be found [here](https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/). This works perfectly, however the main difference is that a SDF is used in Gazebo instead of a URDF.
- `px100_gazebo.launch.py` and `xsarm_gz_classic_copy.launch.py`: Are attempts at spawning the PX100 arm into Gazebo. Both of these launch files can be removed and/or ignored.

## Current Issues
- The `./xacro_models/mariam.urdf.xacro` model is what I've been trying to perfect the most. Ideally, this file will be able to communicate with both ROS and Gazebo. Right now it can spawn into Gazebo and does have a controller that interacts with ROS. However the movement of the joints is strange. The wheels do not spin at the same rate as the model has trouble turning. This can be tested by running `ros2 launch mariam_gazebo mariam_gazebo_spawn_urdf.launch.py` and using the rqt_robot_sterring GUI to drive.
- Interestingly enough, `mariam_gazebo_spawn_sdf.launch.py` exhibits the same behavior. Even though `basic_mobile_robot_gazebo_spawn.launch.py` is proof that using an SDF can have no issues. This implies that the issue in `mariam_gazebo_spawn_urdf.launch.py` isn't the fact that the model format is URDF, but something else...
- Everything is currently setup for simulation. There are no running examples of the navigation stack working on hardware, this is still on the TODO list.