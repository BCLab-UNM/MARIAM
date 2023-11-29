# Summary data for the rover itself and both wheels individually
std_msgs/Header header

float64 linear_setpoint # rover (encoder ticks/sec)
float64 angular_setpoint # rover (encoder ticks/sec)

bridge_msg/PidWheel left_wheel
bridge_msg/PidWheel right_wheel
