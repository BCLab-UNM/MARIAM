// Topic defines
// #define NAMESPACE "monica"
#define NAMESPACE "ross"
#define NODE_NAME "micro_ros_arduino_node_on_" NAMESPACE
#define CMD_VEL_TOPIC_NAME NAMESPACE "/cmd_vel"
#define ODOM_TOPIC_NAME NAMESPACE "/wheel/odom"
#define FORCE_TOPIC_NAME NAMESPACE "/force"
#define LEFT_PID_OUTPUT_TOPIC_NAME NAMESPACE "/left_pid_output"
#define LEFT_SET_POINT_TOPIC_NAME NAMESPACE "/left_set_point"
#define RIGHT_PID_OUTPUT_TOPIC_NAME NAMESPACE "/right_pid_output"
#define RIGHT_SET_POINT_TOPIC_NAME NAMESPACE "/right_set_point"
#define PID_TOPIC_NAME NAMESPACE "/pid"
#define WHEEL_ANALOG_TOPIC_NAME NAMESPACE "/wheel_analog"

// Misc libraries
#include <micro_ros_arduino.h>
#include <stdio.h>

// ROS2 functions
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/time.h>

// ROS2 msgs
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>

// Motor control classes
#include <Movement.h>
#include <Encoder.h> // @TODO Test with and without this above #define ENCODER_OPTIMIZE_INTERRUPTS
#include <PID_v1.h>

// Pin assignments (VNH5019 Motor Driver Carrier)
byte rightDirectionB = A9; // "counterclockwise" input
byte rightDirectionA = A8; // "clockwise" input
byte rightSpeedPin = 3; // PWM input
byte leftDirectionB = A7; // "counterclockwise" input
byte leftDirectionA = A6; // "clockwise" input
byte leftSpeedPin = 4; // PWM input

// Pin assignments for odometry (8400 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 6;
byte leftEncoderA = 12;
byte leftEncoderB = 11;

// Movement and Encoder classes manage motor control and encoder readings
Movement move = Movement(
  rightSpeedPin, rightDirectionA, rightDirectionB, 
  leftSpeedPin, leftDirectionA, leftDirectionB
);
Encoder leftEncoder(leftEncoderA, leftEncoderB);
Encoder rightEncoder(rightEncoderA, rightEncoderB);

// Declare publishers
rcl_publisher_t odom_publisher;
rcl_publisher_t force_publisher;
rcl_publisher_t left_pid_output_publisher;
rcl_publisher_t left_set_point_publisher;
rcl_publisher_t right_pid_output_publisher;
rcl_publisher_t right_set_point_publisher;

std_msgs__msg__Int32 left_ticks;
std_msgs__msg__Int32 right_ticks;
double left_current_speed, right_current_speed;
std_msgs__msg__Float32 force_msg;
double left_set_point, right_set_point, left_pid_output, right_pid_output;
double Kp=500.0, Ki=2000.0, Kd=0.0;

// PID object takes the parameters PID(Input, Output, Setpoint, Kp, Ki, Kd, DIRECT)
PID left_PID(&left_current_speed, &left_pid_output, &left_set_point, Kp, Ki, Kd, DIRECT);
PID right_PID(&right_current_speed, &right_pid_output, &right_set_point, Kp, Ki, Kd, DIRECT);

// Declare subscribers
rcl_subscription_t subscriber_cmd_vel;
rcl_subscription_t subscriber_pid;
rcl_subscription_t subscriber_wheel_analog;

// Declare executors
rclc_executor_t executor;
rclc_executor_t executor_sub_cmd_vel;
rclc_executor_t executor_sub_pid;
rclc_executor_t executor_sub_wheel_analog;

// Declare misc
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

// Create ros message object
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist twist_msg;

// Physical measurements in meters
const double wheel_base = 0.3397; // Distance between left and right wheels
const double wheel_circumference = 0.3613;
const double ticks_per_rotation = 8400; 
double theta_heading = 0;
double previous_x_pos = 0.0;
double previous_y_pos = 0.0;

bool estop = true;

// Indicator light
#define LED_PIN 13

// Global variable to store the timestamp of the last /cmd_vel message
rcl_time_point_value_t last_cmd_vel_time = 0;
rcl_time_point_value_t last_wheel_analog_time = 0;

// =============================================================
// =================== FUNCTION DECLARATIONS ===================
// =============================================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(100);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(100);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(100);
  asm volatile("ldr pc, =0x00000000"); //resets the program counter to the begining like a hardware reset

  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

double ticksToMeters(long ticks) {
  return (ticks / (double)ticks_per_rotation) * wheel_circumference;
}

double thetaToDiff(double theta) {
  return theta * wheel_base;
}

void update_odometry(int left_ticks, int right_ticks, double elapsed_time) {
  // Convert ticks to distance in meters for each wheel
  double left_distance = (wheel_circumference * left_ticks) / ticks_per_rotation;
  double right_distance = (wheel_circumference * right_ticks) / ticks_per_rotation;

  // Calculate average distance and change in heading
  double distance = (left_distance + right_distance) / 2.0;
  double delta_theta = (right_distance - left_distance) / wheel_base;

  // Update the robot's heading
  theta_heading += delta_theta;

  // Calculate the change in position
  double delta_x = distance * cos(theta_heading);
  double delta_y = distance * sin(theta_heading);

  // Update the odometry position using the previous position
  previous_x_pos += delta_x;
  previous_y_pos += delta_y;
  odom_msg.pose.pose.position.x = previous_x_pos;
  odom_msg.pose.pose.position.y = previous_y_pos;

  // Update orientation as a quaternion (flat ground)
  odom_msg.pose.pose.orientation.w = cos(theta_heading / 2.0);  // w component
  odom_msg.pose.pose.orientation.x = 0;                          // x component (no roll)
  odom_msg.pose.pose.orientation.y = 0;                          // y component (no pitch)
  odom_msg.pose.pose.orientation.z = sin(theta_heading / 2.0);  // z component

  // Calculate linear and angular velocities
  odom_msg.twist.twist.linear.x = distance / elapsed_time;
  odom_msg.twist.twist.angular.z = delta_theta / elapsed_time;
}

// =============================================================
// ========================= CALLBACKS =========================
// =============================================================

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){

  // Only run if timer is available
  if (timer != NULL) {

    // Get the current time
    rcl_time_point_value_t current_time;
    rcutils_system_time_now(&current_time);

    // Convert elapsed time from nanoseconds to seconds
    double elapsed_time_sec = last_call_time / 1e9;

    // Saftey e-stop
    // Check if more than 3 seconds have passed since the last /cmd_vel or /wheel_analog message
    if ((current_time - last_cmd_vel_time) > RCL_MS_TO_NS(3000) 
        & (current_time - last_wheel_analog_time) > RCL_MS_TO_NS(3000)) {
      left_set_point = 0;
      right_set_point = 0;
      move.stop(); 
      estop = true; 
    }

    // Read ticks since last callback
    left_ticks.data = leftEncoder.readAndReset(); 
    right_ticks.data = -rightEncoder.readAndReset();

    // Calculate the current speed
    left_current_speed = ticksToMeters(left_ticks.data) / elapsed_time_sec;
    right_current_speed = ticksToMeters(right_ticks.data) / elapsed_time_sec;

    // Update left_pid_output and right_pid_output values via PID
    left_PID.Compute();
    right_PID.Compute();

    // Move robot based on PID output values
    if ((left_pid_output == 0 && right_pid_output == 0)||estop) { move.stop(); }
    else if (left_pid_output >= 0 && right_pid_output >= 0) { move.forward(left_pid_output, right_pid_output); }
    else if (left_pid_output <= 0 && right_pid_output <= 0) { move.backward(left_pid_output*-1, right_pid_output*-1); }
    else if (left_pid_output <= 0 && right_pid_output >= 0) { move.rotateLeft(left_pid_output*-1, right_pid_output); }
    else { move.rotateRight(left_pid_output, right_pid_output*-1); }  

    // Update odom reading
    update_odometry(left_ticks.data, right_ticks.data, elapsed_time_sec);

    // Collect force data
    float raw_force_data = analogRead(A5) * (5.0 / 1023.0);

    // Determine force data using fitted curve
    float coef_0 = 1.0; // coef for x^0
    float coef_1 = 1.0; // coef for x^1
    float coef_2 = 1.0; // coef for x^2
    float coef_3 = 1.0; // coef for x^3
    force_msg.data = coef_3 * pow(raw_force_data, 3) 
                + coef_2 * pow(raw_force_data, 2) 
                + coef_1 * raw_force_data
                + coef_0;

    // Publish force and odom values to ROS2 network
    RCSOFTCHECK(rcl_publish(&force_publisher, &force_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    
    // Publish speeds to ROS2 network
    std_msgs__msg__Float32 left_pid_output_to_publish; left_pid_output_to_publish.data = left_pid_output;
    RCSOFTCHECK(rcl_publish(&left_pid_output_publisher, &left_pid_output_to_publish, NULL));

    // std_msgs__msg__Float32 left_set_point_to_publish; left_set_point_to_publish.data = left_set_point;
    // RCSOFTCHECK(rcl_publish(&left_set_point_publisher, &left_set_point_to_publish, NULL)); 

    std_msgs__msg__Float32 right_pid_output_to_publish; right_pid_output_to_publish.data = right_pid_output;
    RCSOFTCHECK(rcl_publish(&right_pid_output_publisher, &right_pid_output_to_publish, NULL));   

    // std_msgs__msg__Float32 right_set_point_to_publish; right_set_point_to_publish.data = right_set_point;
    // RCSOFTCHECK(rcl_publish(&right_set_point_publisher, &right_set_point_to_publish, NULL)); 
  }
}

void subscription_cmd_vel_callback(const void *msgin) {

  // Read in callback msg
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Update the timestamp of the last received command
  rcutils_system_time_now(&last_cmd_vel_time);
  
  // If velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);

  double yaw = msg->angular.z;
  double linear_sp = msg->linear.x;
  double angular_sp = thetaToDiff(msg->angular.z);
  left_set_point = msg->linear.x - angular_sp;
  right_set_point = msg->linear.x + angular_sp;

  // Activate e-stop if /cmd_vel is 0 overall
  if((msg->linear.x == 0) && (msg->angular.z == 0)) { 
    move.stop(); 
    estop = true; 
  }
  else { estop = false; }
}

void subscription_pid_callback(const void *msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;

  // Set PID parameters
  left_PID.SetTunings(msg->x, msg->y, msg->z);
  right_PID.SetTunings(msg->x, msg->y, msg->z);
}

void subscription_wheel_analog_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  // Update the timestamp of the last received command
  rcutils_system_time_now(&last_wheel_analog_time);

  // Set both motor speeds to the received value, ensuring they are clamped between 0 and 255
  int motor_speed = msg->data;
  if (motor_speed < 0) {
      motor_speed = 0;
  } else if (motor_speed > 255) {
      motor_speed = 255;
  }

  left_set_point = motor_speed;  // Set left speed
  right_set_point = motor_speed; // Set right speed
}

// =============================================================
// ===================== MAIN CONTROL LOOP =====================
// =============================================================

// Initial microcontroller setup
void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  

  // Zero encoders
  leftEncoder.write(0); 
  rightEncoder.write(0);

  // Set odom defaults
  odom_msg.header.frame_id.data = "odom";
  odom_msg.child_frame_id.data = "base_footprint";
  odom_msg.pose.pose.position.z = 0;

  // Wait
  delay(2000);

  // An allocator facilitates dynamic memoory management
  allocator = rcl_get_default_allocator();

  // Set initial time for last command received
  rcutils_system_time_now(&last_cmd_vel_time);
  rcutils_system_time_now(&last_wheel_analog_time);

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create ROS2 node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // Create publishers
  // Left PID output publisher
  RCCHECK(rclc_publisher_init_default(
    &left_pid_output_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), LEFT_PID_OUTPUT_TOPIC_NAME));

  // Left set point publisher
  // RCCHECK(rclc_publisher_init_default(
  //   &left_set_point_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), LEFT_SET_POINT_TOPIC_NAME));

  // Right PID output publisher
  RCCHECK(rclc_publisher_init_default(
    &right_pid_output_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), RIGHT_PID_OUTPUT_TOPIC_NAME));

  // Right set point publisher
  // RCCHECK(rclc_publisher_init_default(
  //   &right_set_point_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), RIGHT_SET_POINT_TOPIC_NAME));

  // Force publsisher
  RCCHECK(rclc_publisher_init_default(
    &force_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), FORCE_TOPIC_NAME));

  // Wheel odom publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC_NAME));

  // Create subscribers
  // Command velocity subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC_NAME));

  // PID parameter subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_pid,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), PID_TOPIC_NAME));

  // Wheel analog subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_wheel_analog,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), WHEEL_ANALOG_TOPIC_NAME));

  // Create timer,
  const unsigned int timer_timeout = 30; // In miliseconds /// @TODO update this to slow down the publishers and test this `ros2 topic hz /Monica/right_ticks`
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initiate tick data
  left_ticks.data = 0;
  right_ticks.data = 0;

  // Set PID parameters
  left_PID.SetMode(AUTOMATIC);
  right_PID.SetMode(AUTOMATIC);
  left_PID.SetOutputLimits(-255,255);
  right_PID.SetOutputLimits(-255,255);
  
  // Create executors
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub_cmd_vel, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub_pid, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub_wheel_analog, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer)); 
  RCCHECK(rclc_executor_add_subscription(&executor_sub_cmd_vel, &subscriber_cmd_vel, &twist_msg, &subscription_cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub_pid, &subscriber_pid, &twist_msg, &subscription_pid_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub_wheel_analog, &subscriber_wheel_analog, &twist_msg, &subscription_wheel_analog_callback, ON_NEW_DATA));
}

// Functions to be looped continiously
// This results in close to a publish rate of 1khz and a sub rate of 100hz
void loop() {

  // Spin for in ns
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); 

  // Read cmd vel for 1ns then swich back
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub_cmd_vel, RCL_MS_TO_NS(1))); 

  // Read pid values for 1ns then swich back
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub_pid, RCL_MS_TO_NS(10))); 
}
