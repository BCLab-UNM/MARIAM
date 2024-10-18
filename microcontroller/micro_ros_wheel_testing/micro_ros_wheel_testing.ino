#include <micro_ros_arduino.h>
#include <Movement.h>
#include <Encoder.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
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

// ROS 2 node, publishers, and messages
rcl_node_t node;
rcl_publisher_t left_total_ticks_publisher;
rcl_publisher_t right_total_ticks_publisher;
rcl_publisher_t left_current_ticks_publisher;
rcl_publisher_t right_current_ticks_publisher;
rcl_publisher_t wheel_velocity_publisher;

std_msgs__msg__Int32 left_wheel_total_ticks_msg;
std_msgs__msg__Int32 right_wheel_total_ticks_msg;
std_msgs__msg__Int32 left_current_ticks_msg;
std_msgs__msg__Int32 right_current_ticks_msg;
geometry_msgs__msg__Twist wheel_velocity_msg;

// ROS 2 support structures
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// Running totals for encoder ticks
long left_total_ticks = 0;
long right_total_ticks = 0;

// Store the last callback time
int64_t last_call_time = 0;

// Helper function to convert ticks to meters
double ticksToMeters(long ticks) {
    return (ticks / (double)TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE;
}

// Timer callback to update and publish velocities and total ticks
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;

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

    // Publish current encoder ticks (not accumulated)
    left_current_ticks_msg.data = left_ticks;  // Use readAndReset value
    right_current_ticks_msg.data = right_ticks; // Use readAndReset value

    rcl_publish(&left_current_ticks_publisher, &left_current_ticks_msg, NULL);
    rcl_publish(&right_current_ticks_publisher, &right_current_ticks_msg, NULL);

    // Calculate elapsed time since last call
    int64_t current_time = rcl_get_time_now();
    double elapsed_time_seconds = (current_time - last_call_time) / 1e9; // Convert nanoseconds to seconds
    last_call_time = current_time; // Update the last call time

    // Calculate velocities in meters per second
    double left_wheel_velocity = ticksToMeters(left_ticks) / elapsed_time_seconds;  // m/s
    double right_wheel_velocity = ticksToMeters(right_ticks) / elapsed_time_seconds; // m/s

    // Publish velocities as a Twist message
    wheel_velocity_msg.linear.x = (left_wheel_velocity + right_wheel_velocity) / 2; // Average linear velocity
    wheel_velocity_msg.angular.z = (right_wheel_velocity - left_wheel_velocity) / WHEEL_CIRCUMFERENCE; // Angular velocity

    rcl_publish(&wheel_velocity_publisher, &wheel_velocity_msg, NULL);
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

    // Initialize current tick publishers
    rclc_publisher_init_default(
        &left_current_ticks_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/current_ticks/left");

    rclc_publisher_init_default(
        &right_current_ticks_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/current_ticks/right");

    // Initialize wheel velocity publisher
    rclc_publisher_init_default(
        &wheel_velocity_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/wheel_vel");

    // Create timer for publishing velocities and ticks at 30ms intervals
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(30), timer_callback);

    // Create executor to manage the timer callback
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    // Start motors at full speed (uncomment to enable)
    // move.forward(255, 255);  // Full speed forward
}

void loop() {
    // Spin the executor to handle the timer callback
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
