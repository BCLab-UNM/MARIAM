#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

class DistributedParameterizer : public rclcpp::Node
{
public:
  DistributedParameterizer() : Node("distributed_parameterizer")
  {
    // Declare and get parameters from config file
    this->declare_parameter("mass", 1.0);
    this->declare_parameter("damping", 10.0);
    this->declare_parameter("stiffness", 100.0);
    this->declare_parameter("frequency", 0.002);  // Timer period in seconds
    this->declare_parameter("x_pos", 0.0);
    this->declare_parameter("y_pos", 0.0);
    this->declare_parameter("z_pos", 0.0);
    
    // Get parameter values
    mass_value_ = this->get_parameter("mass").as_double();
    damping_value_ = this->get_parameter("damping").as_double();
    stiffness_value_ = this->get_parameter("stiffness").as_double();
    double timer_period = this->get_parameter("frequency").as_double();
    double default_x = this->get_parameter("x_pos").as_double();
    double default_y = this->get_parameter("y_pos").as_double();
    double default_z = this->get_parameter("z_pos").as_double();
    
    // Create publishers
    mass_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "admittance_control/mass", 10);
    damping_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "admittance_control/damping", 10);
    stiffness_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "admittance_control/stiffness", 10);
    pose_updater_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
      "admittance_control/px100_virtual_pose_updater", 10);
    
    // Create subscribers
    slam_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "slam_pose", 10,
      std::bind(&DistributedParameterizer::SlamPoseCallback, this, std::placeholders::_1));
      
    force_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "force", 10,
      std::bind(&DistributedParameterizer::ForceCallback, this, std::placeholders::_1));
      
    agent_displacement_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "agent_displacement", 10,
      std::bind(&DistributedParameterizer::AgentDisplacementCallback, this, std::placeholders::_1));
    
    // Create timer with configurable frequency
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&DistributedParameterizer::TimerCallback, this));
    
    // Initialize pose message with config values
    current_virtual_pose_.position.x = default_x;
    current_virtual_pose_.position.y = default_y;
    current_virtual_pose_.position.z = default_z;
    current_virtual_pose_.orientation.x = 0.0;
    current_virtual_pose_.orientation.y = 0.0;
    current_virtual_pose_.orientation.z = 0.0;
    current_virtual_pose_.orientation.w = 1.0;
    
    current_force_ = 0.0;
    current_agent_displacement_ = 0.0;
    
    // Initialize position errors
    current_position_errors_.x = 0.0;
    current_position_errors_.y = 0.0;
    current_position_errors_.z = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "DistributedParameterizer node initialized");
    RCLCPP_INFO(this->get_logger(), "Timer period: %.3f seconds (%.1f Hz)", 
               timer_period, 1.0/timer_period);
    RCLCPP_INFO(this->get_logger(), "Mass: %.2f, Damping: %.2f, Stiffness: %.2f",
               mass_value_, damping_value_, stiffness_value_);
    RCLCPP_INFO(this->get_logger(), "Default pose: [%.3f, %.3f, %.3f]",
               default_x, default_y, default_z);
  }

private:
  void SlamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_virtual_pose_ = msg->pose.pose;  // Note the extra .pose
    
    // Extract covariance matrix (6x6 = 36 elements)
    // Layout: [x, y, z, roll, pitch, yaw] 
    auto covariance = msg->pose.covariance;
    
    // Extract position errors (standard deviations) and store in Vector3
    current_position_errors_.x = sqrt(covariance[0]);   // x variance
    current_position_errors_.y = sqrt(covariance[7]);   // y variance  
    current_position_errors_.z = sqrt(covariance[14]);  // z variance
    
    RCLCPP_DEBUG(this->get_logger(), 
                "SLAM pose: [%.3f, %.3f, %.3f], errors: [%.3f, %.3f, %.3f]",
                current_virtual_pose_.position.x, current_virtual_pose_.position.y, current_virtual_pose_.position.z,
                current_position_errors_.x, current_position_errors_.y, current_position_errors_.z);
  }
  
  void ForceCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_force_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Received force: %.3f", current_force_);
  }
  
  void AgentDisplacementCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    current_agent_displacement_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Received agent displacement: %.3f", current_agent_displacement_);
  }
  
  void TimerCallback()
  {
    // Calculate dynamic stiffness using equation: K = 100 + 10*e + 2*u
    // where e = current_agent_displacement_, u = current_position_errors_.x
    double calculated_stiffness = 100.0 + (10.0 * current_agent_displacement_) + (2.0 * current_position_errors_.x);
    
    // Calculate dynamic x position using equation: x = 23cm + 0.5*e + 0.01*u
    // where e = current_agent_displacement_, u = current_position_errors_.x
    double calculated_x = 0.23 + (0.5 * current_agent_displacement_) + (0.01 * current_position_errors_.x);
    
    // Update current pose with calculated x position
    current_virtual_pose_.position.x = calculated_x;
    
    // Create and populate messages
    auto mass_msg = std_msgs::msg::Float64();
    auto damping_msg = std_msgs::msg::Float64();
    auto stiffness_msg = std_msgs::msg::Float64();
    
    mass_msg.data = mass_value_;
    damping_msg.data = damping_value_;
    stiffness_msg.data = calculated_stiffness;
    
    // Publish all parameters
    mass_pub_->publish(mass_msg);
    damping_pub_->publish(damping_msg);
    stiffness_pub_->publish(stiffness_msg);
    pose_updater_pub_->publish(current_virtual_pose_);
    
    // Optional: Log periodically (every 500 calls = 1 second at 500Hz)
    // static int counter = 0;
    // if (++counter >= 500) {
    //   RCLCPP_INFO(this->get_logger(), 
    //               "Publishing - Mass: %.2f, Damping: %.2f, Stiffness: %.2f, X: %.3f (e=%.3f, u=%.3f)",
    //               mass_value_, damping_value_, calculated_stiffness, calculated_x,
    //               current_agent_displacement_, current_position_errors_.x);
    //   counter = 0;
    // }
  }
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mass_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr damping_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stiffness_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_updater_pub_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr agent_displacement_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Hardcoded parameter values
  double mass_value_;
  double damping_value_;
  double stiffness_value_;
  
  // Storage for most recent messages
  geometry_msgs::msg::Pose current_virtual_pose_;
  float current_force_;
  double current_agent_displacement_;
  geometry_msgs::msg::Vector3 current_position_errors_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistributedParameterizer>());
  rclcpp::shutdown();
  return 0;
}