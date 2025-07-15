# mariam_navigation

This package containts logic for navigating and moving the agents. This includes configuration files for Nav2, a robot following script for Ross, and several hard-coded motions for both robots.

#### Naming Conventions
- `dual` indicates that commands are sent to both robots.
- `linear` indicates that the motion is only along the x-axis.
- `constant` or `sinusoidal` indicates the behavior of the value to cmd_vel.
- `open_loop` or `closed_loop` indicates if there is feedback to keep the robot on the x-axis.

## Fixed Motion Nodes
Details about each node, the motion is creates, and their behavior. Units are in meters and seconds. These nodes perform a fixed motion and then quit after it is complete.
1. `dual_linear_constant_velocity_open_loop.cpp`: Drives in a straight line without feedback at constant velocity. Calculates time to run given distance and speed.
    - @param speed: linear velocity in the x direction
    - @param distance: distance until the robot stops
    - Example: `ros2 run mariam_navigation dual_linear_constant_velocity_open_loop --ros-args -p distance:=2.0 -p speed:=0.2`
2. `dual_linear_constant_velocity_closed_loop.cpp`: Drives in a straight line and uses a PID controller that updates angular velocity to maintain heading along the x-axis. Stops at a given distance by reading the /wheel/odom topic. (PID NOT TUNED, OVERSHOOTING/OSCILLATING)
    - @param speed: linear velocity in the x direction
    - @param distance: distance until the robot stops
    - Example: `ros2 run mariam_navigation dual_linear_constant_velocity_closed_loop --ros-args -p distance:=2.0 -p speed:=0.2`
    - TODO: PID NOT TUNED
3. `dual_linear_sinusoidal_velocity_open_loop.cpp`: Drives in a straight line without feedback at sinusoidal velocity. Stops at a given distance by reading the x position of the /wheel/odom topic.
    - @param min_speed: minimum linear velocity in the x direction of the sin wave
    - @param max_speed: maximum linear velocity in the x direction of the sin wave
    - @param wavelength: time between peaks of the sin wave
    - @param distance: distance until the robot stops
    - Example: `ros2 run mariam_navigation dual_linear_sinusoidal_velocity_open_loop --ros-args -p distance:=2.0 -p max_speed:=0.2 -p min_speed:=0.05`
4. `dual_linear_sinusoidal_acceleration_open_loop.cpp`: Drives in a straight line without feedback at sinusoidal acceleration. Stops at a given distance by reading the x position of the /wheel/odom topic.
    - @param min_accel: minimum linear velocity in the x direction of the sin wave
    - @param max_accel: maximum linear velocity in the x direction of the sin wave
    - @param wavelength: time between peaks of the sin wave
    - @param distance: distance until the robot stops
    - @param initial_velocity: starting velocity
    - @param max_velocity: maximum allowed velocity
    - Example: `ros2 run your_package_name dual_linear_sinusoidal_acceleration_open_loop --ros-args -p distance:=3.0 -p max_acceleration:=0.8 -p min_acceleration:=0.2 -p wavelength:=5.0 -p initial_velocity:=0.1 -p max_velocity:=1.5`
    - TODO: PARAMETERS NOT TUNED

## Other Motion Nodes
Details about each node, the motion is creates, and their behavior. Units are in meters and seconds. These motion nodes are more dynamic and have more complicated behavior.
1. `dual_waypoint_publisher.cpp`: Simlutanously two waypoints 2.0 meters away for Nav2 navigation. The path taken to the waypoints are not guaranteed and varies greatly.
2. `robot_follower.cpp`: 