#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "pid_controller.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>

class CoopTrajPID : public rclcpp::Node
{
public:
  CoopTrajPID() : Node("coop_traj_pid") {
    // change QoS settings
    auto qos_profile = rclcpp::QoS(10);
    qos_profile.best_effort();
    qos_profile.durability_volatile();

    ross_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ross/cmd_vel", qos_profile);
    
      monica_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/monica/cmd_vel", qos_profile);

    base1_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/base1_pose",
      qos_profile,
      std::bind(&CoopTrajPID::control_loop, this, std::placeholders::_1)
    );
    
    base2_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/base2_pose",
      qos_profile,
      std::bind(&CoopTrajPID::control_loop, this, std::placeholders::_1)
    );
    
    base1_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/world_ross_pose",
      qos_profile,
      std::bind(&CoopTrajPID::control_loop, this, std::placeholders::_1)
    );
    
    base2_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/world_ross_pose",
      qos_profile,
      std::bind(&CoopTrajPID::control_loop, this, std::placeholders::_1)
    );

    // Create base 1 PID controllers
    // Declare PID parameters for three controllers
    this->declare_parameter("base1_pid_x_kp", 1.0);
    this->declare_parameter("base1_pid_x_ki", 0.0);
    this->declare_parameter("base1_pid_x_kd", 0.0);

    this->declare_parameter("base1_pid_y_kp", 1.0);
    this->declare_parameter("base1_pid_y_ki", 0.0);
    this->declare_parameter("base1_pid_y_kd", 0.0);

    this->declare_parameter("base1_pid_theta_kp", 1.0);
    this->declare_parameter("base1_pid_theta_ki", 0.0);
    this->declare_parameter("base1_pid_theta_kd", 0.0);

    double base1_kp_x = this->get_parameter("base1_pid_x_kp").as_double();
    double base1_ki_x = this->get_parameter("base1_pid_x_ki").as_double();
    double base1_kd_x = this->get_parameter("base1_pid_x_kd").as_double();

    double base1_kp_y = this->get_parameter("base1_pid_y_kp").as_double();
    double base1_ki_y = this->get_parameter("base1_pid_y_ki").as_double();
    double base1_kd_y = this->get_parameter("base1_pid_y_kd").as_double();

    double base1_kp_theta = this->get_parameter("base1_pid_theta_kp").as_double();
    double base1_ki_theta = this->get_parameter("base1_pid_theta_ki").as_double();
    double base1_kd_theta = this->get_parameter("base1_pid_theta_kd").as_double();

    base1_pid_x_ = std::make_unique<PIDController>(
      base1_kp_x, base1_ki_x, base1_kd_x);
    base1_pid_y_ = std::make_unique<PIDController>(
      base1_kp_y, base1_ki_y, base1_kd_y);
    base1_pid_theta_ = std::make_unique<PIDController>(
      base1_kp_theta, base1_ki_theta, base1_kd_theta);

    // Create base 2 PID controllers
    // Declare PID parameters for three controllers
    this->declare_parameter("base2_pid_x_kp", 1.0);
    this->declare_parameter("base2_pid_x_ki", 0.0);
    this->declare_parameter("base2_pid_x_kd", 0.0);

    this->declare_parameter("base2_pid_y_kp", 1.0);
    this->declare_parameter("base2_pid_y_ki", 0.0);
    this->declare_parameter("base2_pid_y_kd", 0.0);

    this->declare_parameter("base2_pid_theta_kp", 1.0);
    this->declare_parameter("base2_pid_theta_ki", 0.0);
    this->declare_parameter("base2_pid_theta_kd", 0.0);

    double base2_kp_x = this->get_parameter("base2_pid_x_kp").as_double();
    double base2_ki_x = this->get_parameter("base2_pid_x_ki").as_double();
    double base2_kd_x = this->get_parameter("base2_pid_x_kd").as_double();

    double base2_kp_y = this->get_parameter("base2_pid_y_kp").as_double();
    double base2_ki_y = this->get_parameter("base2_pid_y_ki").as_double();
    double base2_kd_y = this->get_parameter("base2_pid_y_kd").as_double();

    double base2_kp_theta = this->get_parameter("base2_pid_theta_kp").as_double();
    double base2_ki_theta = this->get_parameter("base2_pid_theta_ki").as_double();
    double base2_kd_theta = this->get_parameter("base2_pid_theta_kd").as_double();

    base2_pid_x_ = std::make_unique<PIDController>(
      base2_kp_x, base2_ki_x, base2_kd_x);
    base2_pid_y_ = std::make_unique<PIDController>(
      base2_kp_y, base2_ki_y, base2_kd_y);
    base2_pid_theta_ = std::make_unique<PIDController>(
      base2_kp_theta, base2_ki_theta, base2_kd_theta);
    
    // Create timer for control loop
    this->declare_parameter("dt", 0.01); // default 10ms
    dt_ = this->get_parameter("dt").as_double();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&CoopTrajPID::control_loop, this)
    );
  }

private:
  void control_loop() {
    // if no data has been received
    if (!base1_received_ || !base2_received_) {
      // Wait until both base poses are getting published
      // publish zero velocities to stop the robots
      auto zero_twist = geometry_msgs::msg::Twist();
      ross_cmd_vel_pub_->publish(zero_twist);
      monica_cmd_vel_pub_->publish(zero_twist);
      return;
    }

    // ----------------------------------------------------------------------
    //                                BASE 1
    // ----------------------------------------------------------------------
    // Calculate errors for base 1
    double error_x1 = base1_pose_.position.x - base1_actual_pose_.position.x;
    double error_y1 = base1_pose_.position.y - base1_actual_pose_.position.y;
    double desired_theta1 = 2.0 * std::atan2(
      base1_pose_.orientation.z,
      base1_pose_.orientation.w
    );
    double actual_theta1 = 2.0 * std::atan2(
      base1_actual_pose_.orientation.z,
      base1_actual_pose_.orientation.w
    );
    // Normalize angles to [-pi, pi]
    while (desired_theta1 > M_PI) desired_theta1 -= 2 * M_PI;
    while (desired_theta1 < -M_PI) desired_theta1 += 2 * M_PI;
    while (actual_theta1 > M_PI) actual_theta1 -= 2 * M_PI;
    while (actual_theta1 < -M_PI) actual_theta1 += 2 * M_PI;

    double error_theta1 = desired_theta1 - actual_theta1;
    
    // Compute control signals for base 1
    double control_x1 = base1_pid_x_->compute(error_x1, dt_);
    // TODO: make this match the desired architecture
    double control_y1 = base1_pid_y_->compute(error_y1, dt_);
    double control_theta1 = base1_pid_theta_->compute(error_theta1, dt_) + 
                          0.5 * control_y1;

    // Create Twist message for base 1
    auto cmd_vel1 = geometry_msgs::msg::Twist();
    cmd_vel1.linear.x = control_x1;
    cmd_vel1.angular.w = std::sin(control_theta1 / 2.0);
    cmd_vel1.angular.z = std::cos(control_theta1 / 2.0);

    // ----------------------------------------------------------------------
    //                                BASE 2
    // ----------------------------------------------------------------------
    double error_x2 = base2_pose_.position.x - base2_actual_pose_.position.x;
    double error_y2 = base2_pose_.position.y - base2_actual_pose_.position.y;
    double desired_theta2 = 2.0 * std::atan2(
      base2_pose_.orientation.z,
      base2_pose_.orientation.w
    );
    double actual_theta2 = 2.0 * std::atan2(
      base2_actual_pose_.orientation.z,
      base2_actual_pose_.orientation.w
    );
    // Normalize angles to [-pi, pi]
    while (desired_theta2 > M_PI) desired_theta2 -= 2 * M_PI;
    while (desired_theta2 < -M_PI) desired_theta2 += 2 * M_PI;
    while (actual_theta2 > M_PI) actual_theta2 -= 2 * M_PI;
    while (actual_theta2 < -M_PI) actual_theta2 += 2 * M_PI;

    double error_theta2 = desired_theta2 - actual_theta2;

    // Compute control signals for base 2
    double control_x2 = base2_pid_x_->compute(error_x2, dt_);
    // TODO: make this match the desired architecture
    double control_y2 = base2_pid_y_->compute(error_y2, dt_);
    double control_theta2 = base2_pid_theta_->compute(error_theta2, dt_) +
                          0.5 * control_y2;

    // Create Twist message for base 2
    auto cmd_vel2 = geometry_msgs::msg::Twist();
    cmd_vel2.linear.x = control_x2;
    cmd_vel2.angular.w = std::sin(control_theta2 / 2.0);
    cmd_vel2.angular.z = std::cos(control_theta2 / 2.0);


    // Publish commands
    ross_cmd_vel_pub_->publish(cmd_vel1);
    monica_cmd_vel_pub_->publish(cmd_vel2);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_cmd_vel_pub_;
  // NOTE: base 1 = Ross, base 2 = Monica
  // Subscriptions for desired base poses
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base2_sub_;
  // Subscriptions for actual base poses
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base1_actual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base2_actual_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // PID controllers for base 1
  std::unique_ptr<PIDController> base1_pid_x_;
  std::unique_ptr<PIDController> base1_pid_y_;
  std::unique_ptr<PIDController> base1_pid_theta_;

  // PID controllers for base 2
  std::unique_ptr<PIDController> base2_pid_x_;
  std::unique_ptr<PIDController> base2_pid_y_;
  std::unique_ptr<PIDController> base2_pid_theta_;

  // Desired poses
  geometry_msgs::msg::Pose base1_pose_;
  geometry_msgs::msg::Pose base2_pose_;
  // Actual poses
  geometry_msgs::msg::Pose base1_actual_pose_;
  geometry_msgs::msg::Pose base2_actual_pose_;

  double dt_;
  bool base1_received_ = false;
  bool base2_received_ = false;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoopTrajPID>());
  rclcpp::shutdown();
  return 0;
}
