#include <micro_ros_arduino.h>
#include <Movement.h>
#include <Encoder.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// Constants and configurations
#define NAMESPACE "monica"
#define VELOCITY_TOPIC_NAME NAMESPACE "/wheel_velocity"

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

// ROS 2 node, publisher, and messages
rcl_node_t node;
rcl_publisher_t velocity_publisher;
std_msgs__msg__Float32 left_wheel_velocity_msg;
std_msgs__msg__Float32 right_wheel_velocity_msg;

// ROS 2 support structures
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// Helper function to convert ticks to meters
double ticksToMeters(long ticks) {
  return (ticks / (double)TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE;
}

// Timer callback to update and publish velocities
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer == NULL) return;

  // Get encoder values and reset them
  long left_ticks = leftEncoder.readAndReset();
  long right_ticks = -rightEncoder.readAndReset(); // Right encoder is reversed

  // Calculate velocity (ticks per timer interval to meters per second)
  double left_wheel_velocity = ticksToMeters(left_ticks);
  double right_wheel_velocity = ticksToMeters(right_ticks);

  // Publish velocities
  left_wheel_velocity_msg.data = left_wheel_velocity;
  right_wheel_velocity_msg.data = right_wheel_velocity;

  rcl_publish(&velocity_publisher, &left_wheel_velocity_msg, NULL);
  rcl_publish(&velocity_publisher, &right_wheel_velocity_msg, NULL);
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
  
  // Initialize velocity publisher
  rclc_publisher_init_default(
    &velocity_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    VELOCITY_TOPIC_NAME);

  // Create timer for publishing velocities at 30ms intervals
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(30), timer_callback);

  // Create executor to manage the timer callback
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  // Start motors at full speed
  move.forward(255, 255);  // Full speed forward
}

void loop() {
  // Spin the executor to handle the timer callback
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
