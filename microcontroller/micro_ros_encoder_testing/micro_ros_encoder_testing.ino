// This arduino project counts the number of encoder ticks on 
// the left and right wheels of a robot and publishes them to 
// ROS 2. 

// Topic defines
#define NAMESPACE "monica"
#define NODE_NAME "micro_ros_arduino_node_on_" NAMESPACE
#define ODOM_TOPIC_NAME NAMESPACE "/odom/wheel"

// Required libraries
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <Encoder.h>

// Pin assignments for encoders
byte rightEncoderA = 7;
byte rightEncoderB = 6;
byte leftEncoderA = 12;
byte leftEncoderB = 11;
#define LED_PIN 13 // Indicator light

// Initialize Encoders
Encoder leftEncoder(leftEncoderA, leftEncoderB);
Encoder rightEncoder(rightEncoderA, rightEncoderB);

// Declare publishers
rcl_publisher_t left_ticks_publisher;
rcl_publisher_t right_ticks_publisher;

// Declare messages
std_msgs__msg__Int32 left_ticks;
std_msgs__msg__Int32 right_ticks;

rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

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

// Timer callback to read and publish encoder ticks
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    left_ticks.data = leftEncoder.read();
    right_ticks.data = rightEncoder.read();

    RCSOFTCHECK(rcl_publish(&left_ticks_publisher, &left_ticks, NULL));
    RCSOFTCHECK(rcl_publish(&right_ticks_publisher, &right_ticks, NULL));
  }
}

void setup() {
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  // Initialize ROS 2 support and node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // Create publishers for left and right ticks
  RCCHECK(rclc_publisher_init_default(
    &left_ticks_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    NAMESPACE "/left_ticks"
  ));
  RCCHECK(rclc_publisher_init_default(
    &right_ticks_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    NAMESPACE "/right_ticks"
  ));

  // Create timer to trigger every 30 ms (adjust if needed)
  const unsigned int timer_timeout = 30;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback
  ));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
