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
#define JOINT_STATES_TOPIC_NAME NAMESPACE "/joint_states"

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// Misc libraries
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rmw_microros/rmw_microros.h>

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
#include <sensor_msgs/msg/joint_state.h>

// Motor control classes
#include <Movement.h>
#include <Encoder.h> // @TODO Test with and without this above #define ENCODER_OPTIMIZE_INTERRUPTS
#include <PID_v1.h>

// For tracking the state of the Agent
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

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
rcl_publisher_t joint_state_publisher;

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
sensor_msgs__msg__JointState joint_state_msg;

// Physical measurements in meters
const double wheel_base = 0.3397; // Distance between left and right wheels
const double wheel_circumference = 0.3613;
const double ticks_per_rotation = 8400; 
double theta_heading = 0;
double previous_x_pos = 0.0;
double previous_y_pos = 0.0;
double left_front_wheel_position = 0.0;   // Cumulative rotation in radians
double right_front_wheel_position = 0.0;  // Cumulative rotation in radians
double left_back_wheel_position = 0.0;    // Cumulative rotation in radians
double right_back_wheel_position = 0.0;   // Cumulative rotation in radians

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
  destroy_entities();

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

  // Only run if timer is available and estop was not triggered
  if (timer == NULL) return;

  // Get the current time
  rcl_time_point_value_t current_time;
  rcutils_system_time_now(&current_time);

  // Convert elapsed time from nanoseconds to seconds
  double elapsed_time_sec = last_call_time / 1e9;

  // Read ticks since last callback
  left_ticks.data = leftEncoder.readAndReset(); 
  right_ticks.data = -rightEncoder.readAndReset();

  // Calculate the current speed
  left_current_speed = ticksToMeters(left_ticks.data) / elapsed_time_sec;
  right_current_speed = ticksToMeters(right_ticks.data) / elapsed_time_sec;

  // Update left_pid_output and right_pid_output values via PID
  left_PID.Compute();
  right_PID.Compute();

  // if emergency stop was not triggered
  if (!estop) {
    // Saftey e-stop check
    // Check if more than 3 seconds have passed since the last /cmd_vel or /wheel_analog message
    if ((current_time - last_cmd_vel_time) > RCL_MS_TO_NS(3000) &&
        (current_time - last_wheel_analog_time) > RCL_MS_TO_NS(3000)
      ) {
      // stop the motors
      move.stop();
      left_current_speed = 0;
      right_current_speed = 0;
      
      // set the reference signals (values for cmd_vel) to 0
      left_set_point = 0;
      right_set_point = 0;

      // reset the PID controllers by assigning them to new
      // instances of a PID controller.
      left_PID = PID(
        &left_current_speed,
        &left_pid_output,
        &left_set_point, Kp, Ki, Kd, DIRECT);

      right_PID = PID(
        &right_current_speed,
        &right_pid_output,
        &right_set_point, Kp, Ki, Kd, DIRECT);

      // signal the emergency stop was triggered?
      estop = true;
    }

    // Move robot based on PID output values
    // if the output is zero, stop the motors
    else if (left_pid_output == 0 && right_pid_output == 0)
      move.stop();

    else if (left_pid_output >= 0 && right_pid_output >= 0)
      move.forward(left_pid_output, right_pid_output);
    
    else if (left_pid_output <= 0 && right_pid_output <= 0)
      move.backward(left_pid_output*-1, right_pid_output*-1);
    
    else if (left_pid_output <= 0 && right_pid_output >= 0)
      move.rotateLeft(left_pid_output*-1, right_pid_output);
    
    else
      move.rotateRight(left_pid_output, right_pid_output*-1);
  }

  // Update odom reading
  update_odometry(left_ticks.data, right_ticks.data, elapsed_time_sec);

  // Collect force data
  float raw_force_data = analogRead(A5) * (5.0 / 1023.0);

  // Determine force data using fitted curve
  float coef_0 = 0.035582; // coef for x^3
  float coef_1 = -0.055113; // coef for x^2
  float coef_2 = 0.38334; // coef for x^1
  float coef_3 = -0.057718; // coef for x^0
  force_msg.data = coef_0 * pow(raw_force_data, 3) 
              + coef_1 * pow(raw_force_data, 2) 
              + coef_2 * raw_force_data
              + coef_3;
  
  force_msg.data = max(force_msg.data, 0.0f); // Ensure force_msg.data is positive

  // Update joint positions (cumulative wheel rotations in radians)
  double left_rotation = (2.0 * PI * left_ticks.data) / ticks_per_rotation;
  double right_rotation = (2.0 * PI * right_ticks.data) / ticks_per_rotation;

  // Since you have differential drive with 4 wheels, assuming left/right sides move together
  left_front_wheel_position += left_rotation;
  left_back_wheel_position += left_rotation;
  right_front_wheel_position += right_rotation;
  right_back_wheel_position += right_rotation;

  // Update joint state message for all 4 wheels
  joint_state_msg.position.data[0] = left_front_wheel_position;   // left front
  joint_state_msg.position.data[1] = right_front_wheel_position;  // right front
  joint_state_msg.position.data[2] = left_back_wheel_position;    // left back
  joint_state_msg.position.data[3] = right_back_wheel_position;   // right back

  // Convert m/s to rad/s for velocities
  double wheel_radius = wheel_circumference / (2.0 * PI);
  joint_state_msg.velocity.data[0] = left_current_speed / wheel_radius;   // left front
  joint_state_msg.velocity.data[1] = right_current_speed / wheel_radius;  // right front
  joint_state_msg.velocity.data[2] = left_current_speed / wheel_radius;   // left back
  joint_state_msg.velocity.data[3] = right_current_speed / wheel_radius;  // right back

  // Get corrected epoch time using micro-ROS time sync
  int64_t time_ns = rmw_uros_epoch_nanos();

  // Convert to required format and set timestamps
  builtin_interfaces__msg__Time synchronized_timestamp;
  synchronized_timestamp.sec = (int32_t)(time_ns / 1000000000);
  synchronized_timestamp.nanosec = (uint32_t)(time_ns % 1000000000);

  // Apply synchronized timestamp to both messages
  joint_state_msg.header.stamp = synchronized_timestamp;
  odom_msg.header.stamp = synchronized_timestamp;

  // Publish joint states
  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));

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

  // set the reference signals for the PID controllers
  left_set_point = msg->linear.x - angular_sp;
  right_set_point = msg->linear.x + angular_sp;

  estop = false;
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
// ===================== ENTITY MANAGEMENT =====================
// =============================================================

/**
 * This function creates the ROS2 entities (publishers, subscribers, etc.)
 */
void create_entities() {
  // Zero encoders
  leftEncoder.write(0); 
  rightEncoder.write(0);

  // Set odom defaults
  odom_msg.header.frame_id.data = "odom";
  odom_msg.child_frame_id.data = "base_footprint";
  odom_msg.pose.pose.position.z = 0;

  // Wait
  delay(2000);

  // An allocator facilitates dynamic memory management
  allocator = rcl_get_default_allocator();

  // Set initial time for last command received
  rcutils_system_time_now(&last_cmd_vel_time);
  rcutils_system_time_now(&last_wheel_analog_time);

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create ROS2 node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // Synchronize time with the agent
  const int timeout_ms = 1000;
  rmw_uros_sync_session(timeout_ms);

  // --------------------------------------------------------------------------
  //                    Create publishers
  // --------------------------------------------------------------------------
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

  // Joint state publisher
  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), JOINT_STATES_TOPIC_NAME));

  // --------------------------------------------------------------------------
  //                    Create subscribers
  // --------------------------------------------------------------------------
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

  // Set joint state defaults for 4 wheels
  joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(4 * sizeof(rosidl_runtime_c__String));
  joint_state_msg.name.size = 4;
  joint_state_msg.name.capacity = 4;

  joint_state_msg.name.data[0].data = "drivewheel_left_front_joint";
  joint_state_msg.name.data[0].size = strlen("drivewheel_left_front_joint");
  joint_state_msg.name.data[0].capacity = joint_state_msg.name.data[0].size + 1;

  joint_state_msg.name.data[1].data = "drivewheel_right_front_joint";
  joint_state_msg.name.data[1].size = strlen("drivewheel_right_front_joint");
  joint_state_msg.name.data[1].capacity = joint_state_msg.name.data[1].size + 1;

  joint_state_msg.name.data[2].data = "drivewheel_left_back_joint";
  joint_state_msg.name.data[2].size = strlen("drivewheel_left_back_joint");
  joint_state_msg.name.data[2].capacity = joint_state_msg.name.data[2].size + 1;

  joint_state_msg.name.data[3].data = "drivewheel_right_back_joint";
  joint_state_msg.name.data[3].size = strlen("drivewheel_right_back_joint");
  joint_state_msg.name.data[3].capacity = joint_state_msg.name.data[3].size + 1;

  joint_state_msg.position.data = (double*)malloc(4 * sizeof(double));
  joint_state_msg.position.size = 4;
  joint_state_msg.position.capacity = 4;

  joint_state_msg.velocity.data = (double*)malloc(4 * sizeof(double));
  joint_state_msg.velocity.size = 4;
  joint_state_msg.velocity.capacity = 4;

  joint_state_msg.header.frame_id.data = "base_link";
  
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

/**
 * This function destroys all the ROS2 entities (publishers, subscribers, etc.)
 */
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // destroy publishers
  rcl_publisher_fini(&odom_publisher,             &node);
  rcl_publisher_fini(&force_publisher,            &node);
  rcl_publisher_fini(&left_pid_output_publisher,  &node);
  // rcl_publisher_fini(&left_set_point_publisher,   &node);
  rcl_publisher_fini(&right_pid_output_publisher, &node);
  // rcl_publisher_fini(&right_set_point_publisher,  &node);
  rcl_publisher_fini(&joint_state_publisher,      &node);

  // destroy subscribers
  rcl_subscription_fini(&subscriber_cmd_vel,      &node);
  rcl_subscription_fini(&subscriber_pid,          &node);
  rcl_subscription_fini(&subscriber_wheel_analog, &node);
  
  // destroy timer
  rcl_timer_fini(&timer);
  
  // destroy executors
  rclc_executor_fini(&executor);
  rclc_executor_fini(&executor_sub_cmd_vel);
  rclc_executor_fini(&executor_sub_pid);
  rclc_executor_fini(&executor_sub_wheel_analog);
  
  // destroy the node
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  // free the memory allocated for the joint_state_msg
  free(joint_state_msg.name.data);
  free(joint_state_msg.position.data);
  free(joint_state_msg.velocity.data);
}

// =============================================================
// ===================== MAIN CONTROL LOOP =====================
// =============================================================

// Initial microcontroller setup
void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  state = WAITING_AGENT;

  // create_entities();
}

// Function to be looped continuously
// This results in close to a publish rate of 1khz and a sub rate of 100hz
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT);
      break;

    case AGENT_AVAILABLE:
      create_entities();
      state = AGENT_CONNECTED;
      break;
    
    case AGENT_CONNECTED:
      // ping the agent to verify it is still up
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (state == AGENT_CONNECTED) {
          // Spin for in ns
          RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

          // Read cmd vel for 1ns then swich back
          RCSOFTCHECK(rclc_executor_spin_some(&executor_sub_cmd_vel, RCL_MS_TO_NS(1)));

          // Read pid values for 1ns then swich back
          RCSOFTCHECK(rclc_executor_spin_some(&executor_sub_pid, RCL_MS_TO_NS(10)));
      }
      break;
    
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    
    default:
      break;
  }
}
