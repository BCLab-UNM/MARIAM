# mariam_navigation

This package containts logic for navigating and moving the agents. This includes configuration files for Nav2, a robot followering script for Ross, and several hard-coded motions for both robots.

## Motion Nodes
Details about each node, the motion is creates, and their behavior. Units are in meters and seconds.
1. `dual_waypoint_publisher.cpp`: Simlutanously two waypoints 2.0 meters away for Nav2 navigation.
2. `dual_linear_constant_velocity_open_loop.cpp`: Drives in a straight line without feedback at constant velocity. Calculates time to run given distance and speed.
    - @param speed: linear velocity in the x direction
    - @param distance: distance until the robot stops 
3. `dual_linear_constant_velocity_closed_loop.cpp`: Drives in a straight line and uses a PID controller that updates angular velocity to maintain heading along the x-axis. Stops at a given distance by reading the /wheel/odom topic.
    - @param speed: linear velocity in the x direction
    - @param distance: distance until the robot stops 
    - PID NOT TUNED, OVERSHOOTING
4. `dual_linear_sinusoidal_velocity_open_loop.cpp`: Drives in a straight line without feedback with sinusoidal velocity. Stops at a given distance by reading the /wheel/odom topic.
    - @param min_speed: minimum linear velocity in the x direction of the sin wave
    - @param max_speed: maximum linear velocity in the x direction of the sin wave
    - @param wavelength: time between peaks of the sin wave
    - @param distance: distance until the robot stops
    - TODO: NOT STOPPING AT DISTANCE 