#define NAMESPACE "CartersOrinoco"
#define NODE_NAME "micro_ros_arduino_node_on_" NAMESPACE
#define LEFT_TICKS_TOPIC_NAME NAMESPACE "/left_ticks"
#define RIGHT_TICKS_TOPIC_NAME NAMESPACE "/right_ticks"
#define CMD_VEL_TOPIC_NAME NAMESPACE "/cmd_vel"
#define ODOM_TOPIC_NAME NAMESPACE "/odom"


/* NOTES https://www.pololu.com/product/2502#lightbox-picture0J3752
 * see if we can treat our SwarmieControlBoard as if its the DualVNH5019MotorShield as the eletronics are very close but we dont have the debug or current pins exposed
 * https://www.pololu.com/product/2502
 * https://github.com/BCLab-UNM/Swarmathon-Robot/tree/master/CAD-Files/PCB%20Files/SwarmieControlBoard/SWARMIE%20PCB%20REV%20C/LAYERS%20AND%20SCHEMATIC

#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
md.init();
md.setM1Speed(l);
md.setM2Speed(r);
*/

/*
#include <PID_v1.h>

#define PIN_INPUT 0 //Reading analog input
#define PIN_OUTPUT 3 //PWM output 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID leftPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
+ to setup
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;
  leftPID.SetMode(AUTOMATIC);

// + to callback
  Input = analogRead(PIN_INPUT);
  leftPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
*/


// @TODO need to figure out the Encoders see https://github.com/BCLab-UNM/MARIAM/issues/13

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#include <Movement.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_executor_t executor_sub;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t left_tick_publisher;
rcl_publisher_t right_tick_publisher;
rcl_publisher_t odom_publisher;
std_msgs__msg__Int32 left_ticks;
std_msgs__msg__Int32 right_ticks;
nav_msgs__msg__Odometry odom;
rcl_timer_t timer;

const double wheel_circumference_left = 0.3613;
const double wheel_circumference_right = 0.3613;
const double ticks_per_rotation = 2094.625;
const double wheel_base = 0.3397;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //@TODO make a latch for left_ticks, right_ticks & odom?
    RCSOFTCHECK(rcl_publish(&left_tick_publisher, &left_ticks, NULL));
    RCSOFTCHECK(rcl_publish(&right_tick_publisher, &right_ticks, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom, NULL));
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
  //@TODO make a latch for left_ticks, right_ticks & odom?

  //@TODO replace with encoder data
  // @TODO move this into a timer callback with odom & tf code
  double yaw = msg->angular.z;
  left_ticks.data = (msg->linear.x - yaw * wheel_base / 2.0) * 50;
  right_ticks.data = (msg->linear.x + yaw * wheel_base / 2.0) * 50;
  //@TODO add odom calcs here, and pid compute and sending to motors
}
  
void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &left_tick_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),LEFT_TICKS_TOPIC_NAME));

  RCCHECK(rclc_publisher_init_default(
    &right_tick_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), RIGHT_TICKS_TOPIC_NAME));
    
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC_NAME));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC_NAME));

  // create timer,
  const unsigned int timer_timeout = 0.1;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  left_ticks.data = 0;
  right_ticks.data = 0;

  // create executors
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer)); 
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))); // Spin for 10ns
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1))); // Read cmd vel for 1ns then swich back
  // This results in something close to a publish rate of 1khz and a sub rate of 100hz
}
