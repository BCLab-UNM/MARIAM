#include <micro_ros_arduino.h>
#include <Movement.h>
#include <Encoder.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

// Pin assignments for the motor driver and encoders
byte rightDirectionA = A8;
byte rightDirectionB = A9;
byte rightSpeedPin = 3;
byte leftDirectionA = A6;
byte leftDirectionB = A7;
byte leftSpeedPin = 4;

byte leftEncoderA = 12;
byte leftEncoderB = 11;
byte rightEncoderA = 7;
byte rightEncoderB = 6;

const double WHEEL_CIRCUMFERENCE = 0.3613; // Meters
const int TICKS_PER_ROTATION = 8400; // Encoder ticks per wheel rotation

// Motor and encoder setup
Movement move(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
Encoder leftEncoder(leftEncoderA, leftEncoderB);
Encoder rightEncoder(rightEncoderA, rightEncoderB);

// ROS 2 node, publishers, subscribers, and messages
rcl_node_t node;
rcl_publisher_t left_total_ticks_publisher;
rcl_publisher_t right_total_ticks_publisher;
rcl_publisher_t wheel_velocity_publisher;
rcl_publisher_t force_publisher;
rcl_subscription_t wheel_analog_subscriber;

std_msgs__msg__Int32 left_wheel_total_ticks_msg;
std_msgs__msg__Int32 right_wheel_total_ticks_msg;
geometry_msgs__msg__Twist wheel_velocity_msg;
std_msgs__msg__Int32 analog_msg;  // Changed from Float32 to Int32
std_msgs__msg__Float32 force_msg;

// ROS 2 support structures
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// Running totals for encoder ticks
long left_total_ticks = 0;
long right_total_ticks = 0;

// Global variables for force averaging
float force_samples[30];  // Array to store force readings
int force_sample_count = 0; // Counter for the number of stored samples
int averaging_window = 5;  

// Helper function to convert ticks to meters
double ticksToMeters(long ticks) {
    return (ticks / (double)TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE;
}

// Function for building a list of the most recent force readings
// Used for debugging force values
void add_force_sample(float new_sample) {
  // Shift samples if buffer is full
  if (force_sample_count >= averaging_window) {
    for (int i = 1; i < averaging_window; i++) {
      force_samples[i - 1] = force_samples[i];
    }
    force_samples[averaging_window - 1] = new_sample;
  } else {
    // Add new sample
    force_samples[force_sample_count++] = new_sample;
  }
}

// Function for averaging our forces list
// Used for debugging force values
float compute_average_force() {
  float sum = 0.0;
  int count = force_sample_count < averaging_window ? force_sample_count : averaging_window;
  for (int i = 0; i < count; i++) {
    sum += force_samples[i];
  }
  return count > 0 ? sum / count : 0;
}

// Timer callback to update and publish velocities and total ticks
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;

    // Convert elapsed time from nanoseconds to seconds
    double elapsed_time_sec = last_call_time / 1e9;

    // Get current encoder values and reset them
    long left_ticks = leftEncoder.readAndReset();
    long right_ticks = -rightEncoder.readAndReset(); // Right encoder is reversed

    // Accumulate total ticks
    left_total_ticks += left_ticks;
    right_total_ticks += right_ticks;

    // Publish total encoder ticks
    left_wheel_total_ticks_msg.data = left_total_ticks;
    right_wheel_total_ticks_msg.data = right_total_ticks;

    rcl_publish(&left_total_ticks_publisher, &left_wheel_total_ticks_msg, NULL);
    rcl_publish(&right_total_ticks_publisher, &right_wheel_total_ticks_msg, NULL);

    // Calculate velocities in meters per second using the elapsed time
    double left_wheel_speed = ticksToMeters(left_ticks) / elapsed_time_sec;  // m/s
    double right_wheel_speed = ticksToMeters(right_ticks) / elapsed_time_sec; // m/s

    // Publish velocities as a Twist message
    wheel_velocity_msg.linear.x = (left_wheel_speed + right_wheel_speed) / 2; // Average linear velocity
    wheel_velocity_msg.angular.z = (right_wheel_speed - left_wheel_speed) / WHEEL_CIRCUMFERENCE; // Angular velocity

    // Read new force data and add it to the sample buffer
    float current_force = analogRead(A5) * (5.0 / 1023.0);
    add_force_sample(current_force);

    // Compute the average force and publish it
    force_msg.data = compute_average_force();

    rcl_publish(&force_publisher, &force_msg, NULL);
    rcl_publish(&wheel_velocity_publisher, &wheel_velocity_msg, NULL);
}

// Callback function for wheel_analog subscriber (receiving Int32)
void wheel_analog_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int motor_speed = msg->data;  // Directly use the received Int32 value

    // Ensure the speed is within the valid range [0, 255]
    motor_speed = constrain(motor_speed, 0, 255);

    // Set both motors to the received speed
    move.forward(motor_speed, motor_speed);
}

void setup() {
    // Initialize micro-ROS communication
    set_microros_transports();
  
    // Initialize motor control and encoders
    move.stop();  // Start with motors stopped
    leftEncoder.write(0);
    rightEncoder.write(0);

    // Initialize ROS 2
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "wheel_velocity_node", "", &support);
  
    // Initialize total tick publishers
    rclc_publisher_init_default(
        &left_total_ticks_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/total_wheel_ticks/left");

    rclc_publisher_init_default(
        &right_total_ticks_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/total_wheel_ticks/right");

    // Initialize wheel velocity publisher
    rclc_publisher_init_default(
        &wheel_velocity_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/wheel_vel");

    // Initialize force publisher
    rclc_publisher_init_default(
        &force_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/force");

    // Initialize wheel_analog subscriber (for Int32 type)
    rclc_subscription_init_default(
        &wheel_analog_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/wheel_analog");

    // Create timer for publishing velocities and ticks at 30ms intervals
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(30), timer_callback);

    // Create executor to manage the timer and subscriber callbacks
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &wheel_analog_subscriber, &analog_msg, &wheel_analog_callback, ON_NEW_DATA);
}

void loop() {
    // Spin the executor to handle the timer callback and subscription
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
