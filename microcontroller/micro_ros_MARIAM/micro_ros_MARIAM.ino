#define NAMESPACE "monica"
//#define NAMESPACE "ross"
#define NODE_NAME "micro_ros_arduino_node_on_" NAMESPACE
#define CMD_VEL_TOPIC_NAME NAMESPACE "/cmd_vel"
#define ODOM_TOPIC_NAME NAMESPACE "/odom/wheel"
#define FORCE_TOPIC_NAME NAMESPACE "/force"
#define LEFT_CURRENT_SPEED_TOPIC_NAME NAMESPACE "/pid_left_current_speed"
#define SPEEDL_TOPIC_NAME NAMESPACE "/pid_speedL"
#define LEFT_SP_TOPIC_NAME NAMESPACE "/pid_left_sp"
#define RIGHT_CURRENT_SPEED_TOPIC_NAME NAMESPACE "/pid_right_current_speed"
#define SPEEDR_TOPIC_NAME NAMESPACE "/pid_speedR"
#define RIGHT_SP_TOPIC_NAME NAMESPACE "/pid_right_sp"
#define PID_TOPIC_NAME NAMESPACE "/pid"

/* Prior pid tuning 
Kp=0.0075
Ki=0.15
Kd=0.002
db=0.0
st=0.0
wu=3.0
ff_l=0.0095
ff_r=0.012
*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>

#include <Movement.h>
#include <Encoder.h> // @TODO Test with and without this above #define ENCODER_OPTIMIZE_INTERRUPTS
#include <PID_v1.h>

//Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionB = A9; //"counterclockwise" input
byte rightDirectionA = A8; //"clockwise" input
byte rightSpeedPin = 3; //PWM input
byte leftDirectionB = A7; //"counterclockwise" input
byte leftDirectionA = A6; //"clockwise" input
byte leftSpeedPin = 4; //PWM input

//Odometry (8400 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 6;
byte leftEncoderA = 12;
byte leftEncoderB = 11;
Movement move = Movement(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
Encoder leftEncoder(leftEncoderA,leftEncoderB);
Encoder rightEncoder(rightEncoderA,rightEncoderB);

rcl_publisher_t odom_publisher;
rcl_publisher_t force_publisher;
rcl_publisher_t data_publisher;
rcl_publisher_t left_current_speed_publisher;
rcl_publisher_t speedl_publisher;
rcl_publisher_t left_sp_publisher;
rcl_publisher_t right_current_speed_publisher;
rcl_publisher_t speedr_publisher;
rcl_publisher_t right_sp_publisher;

std_msgs__msg__Int32 left_ticks;
std_msgs__msg__Int32 right_ticks;
double left_current_speed, right_current_speed;
std_msgs__msg__Float32 heading;
std_msgs__msg__Float32 force;
double left_sp, right_sp, dleft, dright, speedL, speedR;
// double Kp=750.0, Ki=0.0, Kd=0.0;
double Kp=500.0, Ki=2000.0, Kd=0.0;//double Kp=0.0075, Ki=0.15, Kd=0.002; //double Kp=2, Ki=5, Kd=1;
PID left_PID(&left_current_speed, &speedL, &left_sp, Kp, Ki, Kd, DIRECT);
PID right_PID(&right_current_speed, &speedR, &right_sp, Kp, Ki, Kd, DIRECT);

const double wheelBase = 0.3397; // OLD:0.278; //distance between left and right wheels (in M)
const double leftWheelCircumference = 0.3613; // Hardwheels in meters
const double rightWheelCircumference = 0.3613; // Hardwheels in meters
const int cpr = 8400; //"cycles per revolution" -- number of encoder increments per one wheel revolution or 2094.625?

rcl_subscription_t subscriber;
rcl_subscription_t subscriber_pid;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_executor_t executor_sub;
rclc_executor_t executor_sub_pid;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

nav_msgs__msg__Odometry odom;
rcl_timer_t timer;
const double ticks_per_rotation = 8400; //2094.625;
double theta_heading = 0;
unsigned int pid_compute_ticks = 0;
bool estop = true;

#define LED_PIN 13

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

double leftTicksToMeters(double leftTicks_arg) {
  return (leftWheelCircumference * (double)leftTicks_arg) / (double)cpr;
}

double rightTicksToMeters(double rightTicks_arg) {
  return (rightWheelCircumference * (double)rightTicks_arg) / (double)cpr;
}

double metersToTicks(double meters) {
  return (meters * cpr) / ((leftWheelCircumference + rightWheelCircumference) / 2);
}

double leftMetersToTicks(double meters) {
  return (meters * cpr) / leftWheelCircumference;
}

double rightMetersToTicks(double meters) {
  return (meters * cpr) / rightWheelCircumference;
}

double diffToTheta(double right, double left) {
  return (right - left) / wheelBase;
}

double thetaToDiff(double theta) {
  return theta * wheelBase;
}


void update_rotation(float x, float y, float z) {
    float c1 = cos(y);
    float c2 = cos(z);
    float c3 = cos(x);
    float s1 = sin(y);
    float s2 = sin(z);
    float s3 = sin(x);
    odom.pose.pose.orientation.w = c1 * c2 * c3 - s1 * s2 * s3;
    odom.pose.pose.orientation.x= s1 * s2 * c3 + c1 * c2 * s3;
    odom.pose.pose.orientation.y = s1 * c2 * c3 + c1 * s2 * s3;
    odom.pose.pose.orientation.z = c1 * s2 * c3 - s1 * c2 * s3;
}

/*
def get_rotation(self):
    q = quaternion_from_euler(0, 0, self.th)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
  */ 

void update_odometry(int ticks_left, int ticks_right){
  dleft = ticks_left / ticks_per_rotation * leftWheelCircumference;
  dright = ticks_right / ticks_per_rotation * rightWheelCircumference;

  double dcenter = (dleft + dright) / 2.0;
  double dtheta = (dright - dleft) / wheelBase;

  double dx = dcenter * cos(theta_heading);
  double dy = dcenter * sin(theta_heading);

  odom.pose.pose.position.x += dx;
  odom.pose.pose.position.y += dy;
  theta_heading += dtheta/2;
  heading.data = theta_heading;

  update_rotation(0, 0, theta_heading);
  //odom.pose.pose.orientation = //get_rotation()

  odom.twist.twist.linear.x = dx;
  odom.twist.twist.linear.y = dy;
  odom.twist.twist.angular.z = dtheta;

  //odom.header.stamp = get_clock().now().to_msg();
  //odom.pose.pose = Pose(position=Point(x=x, y=y, z=0.0), orientation=get_rotation());
} //end update_odometry function

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  if (timer != NULL) {
    //Read & Reset encoders
    left_ticks.data = leftEncoder.readAndReset(); 
    right_ticks.data = -rightEncoder.readAndReset();
    // Meter/Nanosecond ->Meter/second
    left_current_speed =   leftTicksToMeters(left_ticks.data) *(1000000000/last_call_time);
    right_current_speed = rightTicksToMeters(right_ticks.data)*(1000000000/last_call_time);
    left_PID.Compute();
    right_PID.Compute();
    //speedL = left_sp * 50;
    //speedR = right_sp * 50;
    // Feed forward
    //speedL += ff_l * speedL;
    //speedR += ff_r * speedL;
    if ((speedL == 0 && speedR == 0)||estop) { move.stop(); }
    else if (speedL >= 0 && speedR >= 0) { move.forward(speedL, speedR); }
    else if (speedL <= 0 && speedR <= 0) { move.backward(speedL*-1, speedR*-1); }
    else if (speedL <= 0 && speedR >= 0) { move.rotateLeft(speedL*-1, speedR); }
    else { move.rotateRight(speedL, speedR*-1); }  

    update_odometry(left_ticks.data, right_ticks.data);
    force.data = analogRead(A5)
    //force.data = analogRead(A5)* (5.0 / 1023.0); // to voltage  
    //force.data = exp(force.data); 
    //force.data = -0.00000008099*pow(force.data, 5) + 0.000013687*pow(force.data, 4) 
                 - 0.00068314*pow(force.data, 3) + 0.011707*pow(force.data, 2) 
                 - 0.0081636*force.data + 0.0098014; // to force

    RCSOFTCHECK(rcl_publish(&force_publisher, &force, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom, NULL));
    
    std_msgs__msg__Float32 left_current_speed_to_publish; left_current_speed_to_publish.data = left_current_speed;
    RCSOFTCHECK(rcl_publish(&left_current_speed_publisher, &left_current_speed_to_publish, NULL)); 
    std_msgs__msg__Float32 speedl_to_publish; speedl_to_publish.data = speedL;
    RCSOFTCHECK(rcl_publish(&speedl_publisher, &speedl_to_publish, NULL));
    std_msgs__msg__Float32 left_sp_to_publish; left_sp_to_publish.data = left_sp;
    RCSOFTCHECK(rcl_publish(&left_sp_publisher, &left_sp_to_publish, NULL)); 
    std_msgs__msg__Float32 right_current_speed_to_publish; right_current_speed_to_publish.data = right_current_speed;
    RCSOFTCHECK(rcl_publish(&right_current_speed_publisher, &right_current_speed_to_publish, NULL)); 
    std_msgs__msg__Float32 speedr_to_publish; speedr_to_publish.data = speedR;
    RCSOFTCHECK(rcl_publish(&speedr_publisher, &speedr_to_publish, NULL));    
    std_msgs__msg__Float32 right_sp_to_publish; right_sp_to_publish.data = right_sp;
    RCSOFTCHECK(rcl_publish(&right_sp_publisher, &right_sp_to_publish, NULL)); 
    
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
  double yaw = msg->angular.z;
  double linear_sp = msg->linear.x;
  double angular_sp = thetaToDiff(msg->angular.z);
  left_sp = msg->linear.x - angular_sp;
  right_sp = msg->linear.x + angular_sp;
  if((msg->linear.x==0) && (msg->angular.z==0)) { move.stop(); estop = true;}
  else{estop = false;}
}

void subscription_pid_callback(const void *msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  //left_PID.SetTunings(double Kp, double Ki, double Kd, int POn);
  left_PID.SetTunings(msg->x, msg->y, msg->z);
  //right_PID.SetTunings(double Kp, double Ki, double Kd, int POn)
  right_PID.SetTunings(msg->x, msg->y, msg->z);
} // end subscription_pid_callback
  
void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  

  //Reset encoders
  leftEncoder.write(0); 
  rightEncoder.write(0);
  odom.header.frame_id.data = "odom";
  odom.child_frame_id.data = "base_link"; //@TODO add namespaces?
  odom.pose.pose.position.z = 0;

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create publishers

  RCCHECK(rclc_publisher_init_default(
      &left_current_speed_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), LEFT_CURRENT_SPEED_TOPIC_NAME));
      
  RCCHECK(rclc_publisher_init_default(
    &speedl_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), SPEEDL_TOPIC_NAME));
  RCCHECK(rclc_publisher_init_default(
    &left_sp_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), LEFT_SP_TOPIC_NAME));
  RCCHECK(rclc_publisher_init_default(
    &right_current_speed_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), RIGHT_CURRENT_SPEED_TOPIC_NAME));
    
  RCCHECK(rclc_publisher_init_default(
    &speedr_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), SPEEDR_TOPIC_NAME));

  RCCHECK(rclc_publisher_init_default(
    &right_sp_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), RIGHT_SP_TOPIC_NAME));
   
 RCCHECK(rclc_publisher_init_default(
    &force_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), FORCE_TOPIC_NAME));
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ODOM_TOPIC_NAME));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC_NAME));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_pid,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), PID_TOPIC_NAME));  
    

  // create timer,
  const unsigned int timer_timeout = 30; // In miliseconds /// @TODO update this to slow down the publishers and test this `ros2 topic hz /Monica/right_ticks`
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  left_ticks.data = 0;
  right_ticks.data = 0;
  left_PID.SetMode(AUTOMATIC);
  right_PID.SetMode(AUTOMATIC);
  left_PID.SetOutputLimits(-255,255);
  right_PID.SetOutputLimits(-255,255);
  

  // create executors
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub_pid, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub_pid, &subscriber_pid, &msg, &subscription_pid_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer)); 
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Spin for in ns
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1))); // Read cmd vel for 1ns then swich back
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub_pid, RCL_MS_TO_NS(10))); // Read pid values for 1ns then swich back
  // This results in something close to a publish rate of 1khz and a sub rate of 100hz
}
