#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pid_controller.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

class CoopTrajPID : public rclcpp::Node
{
public:
  CoopTrajPID() : Node("coop_traj_pid") {
    marker_to_base.setIdentity();
    marker_to_base.setOrigin(tf2::Vector3(0.23, -0.0-75, -0.06));
    marker_to_base.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // change QoS settings
    auto qos_profile = rclcpp::QoS(10);
    qos_profile.best_effort();
    qos_profile.durability_volatile();

    ross_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ross/cmd_vel", qos_profile);
    
    monica_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/monica/cmd_vel", qos_profile);

    bases_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/desired_poses",
      qos_profile,
      std::bind(&CoopTrajPID::bases_callback, this, std::placeholders::_1)
    );
    
    base1_actual_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/world_ross_pose",
      qos_profile,
      std::bind(&CoopTrajPID::base1_actual_callback, this, std::placeholders::_1)
    );
    
    base2_actual_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/world_ross_pose",
      qos_profile,
      std::bind(&CoopTrajPID::base2_actual_callback, this, std::placeholders::_1)
    );

    this->declare_parameter("max_linear_velocity", 0.2);
    this->declare_parameter("max_angular_velocity", 0.4);

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
      std::chrono::duration<double>(dt_),
      std::bind(&CoopTrajPID::control_loop, this)
    );
  }

private:

  geometry_msgs::msg::Pose transform_pose(const geometry_msgs::msg::Pose &pose) {
    // Convert geometry_msgs::msg::Pose to tf2::Transform
    tf2::Transform pose_tf;
    pose_tf.setOrigin(
      tf2::Vector3(
        pose.position.x, pose.position.y, pose.position.z));
    pose_tf.setRotation(
      tf2::Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w));

    // Apply the transformation
    tf2::Transform transformed_tf = pose_tf * marker_to_base;

    // Convert back to geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose transformed_pose;
    transformed_pose.position.x = transformed_tf.getOrigin().x();
    transformed_pose.position.y = transformed_tf.getOrigin().y();
    transformed_pose.position.z = transformed_tf.getOrigin().z();
    tf2::Quaternion q = transformed_tf.getRotation();
    transformed_pose.orientation.x = q.x();
    transformed_pose.orientation.y = q.y();
    transformed_pose.orientation.z = q.z();
    transformed_pose.orientation.w = q.w();

    return transformed_pose;
  }

  // subscriber callbacks
  void base1_actual_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    base1_actual_pose_ = transform_pose(*msg);
  }

  void base2_actual_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    base2_actual_pose_ = transform_pose(*msg);
  }

  void bases_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.size() >= 2) {
      base1_pose_ = msg->poses[0];
      base2_pose_ = msg->poses[1];
      bases_received_ = true;
    }
  }

  double get_yaw(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

  void control_loop() {
    // if no data has been received
    if (!bases_received_) {
      RCLCPP_WARN(this->get_logger(), "No base poses received yet.");
      // Wait until both base poses are getting published
      // publish zero velocities to stop the robots
      auto zero_twist = geometry_msgs::msg::Twist();
      ross_cmd_vel_pub_->publish(zero_twist);
      monica_cmd_vel_pub_->publish(zero_twist);
      return;
    }

    // ----------------------------------------------------------------------
    //                           BASE 1 (Ross)
    // ----------------------------------------------------------------------
    // Calculate errors for base 1 (world frame)
    double world_error_x1 = base1_pose_.position.x - base1_actual_pose_.position.x;
    double world_error_y1 = base1_pose_.position.y - base1_actual_pose_.position.y;
    double desired_theta1 = get_yaw(base1_pose_.orientation);
    double actual_theta1 = get_yaw(base1_actual_pose_.orientation);
    desired_theta1 = normalize_angle(desired_theta1);
    actual_theta1 = normalize_angle(actual_theta1);

    // transform errors to the body frame
    double error_x1 = world_error_x1 * cos(actual_theta1) + world_error_y1 * sin(actual_theta1);
    double error_y1 = -world_error_x1 * sin(actual_theta1) + world_error_y1 * cos(actual_theta1);
    double error_theta1 = desired_theta1 - actual_theta1;
    
    // Compute control signals for base 1
    double control_x1 = base1_pid_x_->compute(error_x1, dt_);
    // TODO: make this match the desired architecture
    double control_y1 = base1_pid_y_->compute(error_y1, dt_);
    double theta_d1 = error_theta1 + control_y1;
    double control_theta1 = base1_pid_theta_->compute(theta_d1, dt_);

    // Create Twist message for base 1
    auto cmd_vel1 = geometry_msgs::msg::Twist();
    cmd_vel1.linear.x = control_x1;
    cmd_vel1.angular.z = control_theta1;

    // ----------------------------------------------------------------------
    //                           BASE 2 (Monica)
    // ----------------------------------------------------------------------
    // Calculate errors for base 2 (world frame)
    double world_error_x2 = base2_pose_.position.x - base2_actual_pose_.position.x;
    double world_error_y2 = base2_pose_.position.y - base2_actual_pose_.position.y;
    double desired_theta2 = get_yaw(base2_pose_.orientation);
    // Add pi to make the PID controller think the heading is behind monica
    double actual_theta2 = get_yaw(base2_actual_pose_.orientation) + M_PI;
    // Normalize angles to [-pi, pi]
    desired_theta2 = normalize_angle(desired_theta2);
    actual_theta2 = normalize_angle(actual_theta2);

    // transform errors to the body frame
    double error_x2 = world_error_x2 * cos(actual_theta2) + world_error_y2 * sin(actual_theta2);
    double error_y2 = -world_error_x2 * sin(actual_theta2) + world_error_y2 * cos(actual_theta2);
    double error_theta2 = desired_theta2 - actual_theta2;

    // Compute control signals for base 2
    double control_x2 = base2_pid_x_->compute(error_x2, dt_);
    // TODO: make this match the desired architecture
    double control_y2 = base2_pid_y_->compute(error_y2, dt_);
    double theta_d2 = error_theta2 + control_y2;
    double control_theta2 = base2_pid_theta_->compute(theta_d2, dt_);

    // Create Twist message for base 2
    auto cmd_vel2 = geometry_msgs::msg::Twist();
    cmd_vel2.linear.x = control_x2;
    cmd_vel2.angular.z = control_theta2;

    // Print debug info
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Base 1 Desired Pose: x: %.3f, y: %.3f, theta: %.3f", base1_pose_.position.x, base1_pose_.position.y, desired_theta1);
    RCLCPP_INFO(this->get_logger(), "Base 1 Actual Pose: x: %.3f, y: %.3f, theta: %.3f", base1_actual_pose_.position.x, base1_actual_pose_.position.y, actual_theta1);
    RCLCPP_INFO(this->get_logger(), "Base 1 Errors: x: %.3f, y: %.3f, theta: %.3f", error_x1, error_y1, error_theta1);
    RCLCPP_INFO(this->get_logger(), "Base 1 Controls: x: %.3f, theta: %.3f", control_x1, control_theta1);
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "Base 2 Desired Pose: x: %.3f, y: %.3f, theta: %.3f", base2_pose_.position.x, base2_pose_.position.y, desired_theta2);
    RCLCPP_INFO(this->get_logger(), "Base 2 Actual Pose: x: %.3f, y: %.3f, theta: %.3f", base2_actual_pose_.position.x, base2_actual_pose_.position.y, actual_theta2);
    RCLCPP_INFO(this->get_logger(), "Base 2 Errors: x: %.3f, y: %.3f, theta: %.3f", error_x2, error_y2, error_theta2);
    RCLCPP_INFO(this->get_logger(), "Base 2 Controls: x: %.3f, theta: %.3f", control_x2, control_theta2);

    // Publish commands
    ross_cmd_vel_pub_->publish(cmd_vel1);
    monica_cmd_vel_pub_->publish(cmd_vel2);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_cmd_vel_pub_;
  // NOTE: base 1 = Ross, base 2 = Monica
  // Subscriptions for desired base poses
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bases_sub_;
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
  bool bases_received_ = false;

  tf2::Transform marker_to_base;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoopTrajPID>());
  rclcpp::shutdown();
  return 0;
}
