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
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);
    this->declare_parameter("z", 0.707);
    this->declare_parameter("w", 0.707);
    
    // Get parameter values
    mass_value_ = this->get_parameter("mass").as_double();
    damping_value_ = this->get_parameter("damping").as_double();
    stiffness_value_ = this->get_parameter("stiffness").as_double();
    double timer_period = this->get_parameter("frequency").as_double();
    default_x = this->get_parameter("x_pos").as_double();
    default_y = this->get_parameter("y_pos").as_double();
    default_z = this->get_parameter("z_pos").as_double();
    default_orientation_x = this->get_parameter("x").as_double();
    default_orientation_y = this->get_parameter("y").as_double();
    default_orientation_z = this->get_parameter("z").as_double();
    default_orientation_w = this->get_parameter("w").as_double();

    // change QoS settings
    auto qos_profile = rclcpp::QoS(10);
    qos_profile.best_effort();
    qos_profile.durability_volatile();
    
    // Create publishers
    mass_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "admittance_control/mass", qos_profile);
    damping_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "admittance_control/damping", qos_profile);
    stiffness_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "admittance_control/stiffness", qos_profile);
    pose_updater_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
      "admittance_control/px100_virtual_pose_updater", qos_profile);
      
    force_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "force", qos_profile,
      std::bind(&DistributedParameterizer::ForceCallback, this, std::placeholders::_1));
      
    agent_displacement_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "agent_displacement", qos_profile,
      std::bind(&DistributedParameterizer::AgentDisplacementCallback, this, std::placeholders::_1));
    
    // Create timer with configurable frequency
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&DistributedParameterizer::TimerCallback, this));
    
    // Initialize pose message with config values
    current_virtual_pose_.position.x = default_x;
    current_virtual_pose_.position.y = default_y;
    current_virtual_pose_.position.z = default_z;
    current_virtual_pose_.orientation.x = default_orientation_x;
    current_virtual_pose_.orientation.y = default_orientation_y;
    current_virtual_pose_.orientation.z = default_orientation_z;
    current_virtual_pose_.orientation.w = default_orientation_w;

    current_force_ = 0.0;
    current_agent_displacement_error_ = 0.0;
    
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
  
  void ForceCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_force_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Received force: %.3f", current_force_);
  }
  
  void AgentDisplacementCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double manipulator_distance = 0.55;
    current_agent_displacement_error_ = msg->data - manipulator_distance;
    RCLCPP_DEBUG(this->get_logger(), "Received agent displacement error: %.3f", current_agent_displacement_error_);
  }
  
  void TimerCallback()
  {
    // Calculate dynamic stiffness using equation: K = 100 + 10*e + 2*u
    // where e = current_agent_displacement_error_, u = current_position_errors_.x
    double calculated_stiffness = 70.0 + (30.0 * current_agent_displacement_error_);
    
    // Calculate dynamic y position using equation: y = 23cm + 0.5*e + 0.01*u
    // where e = current_agent_displacement_error_, u = current_position_errors_.y
    double calculated_y = 0.25 + (0.5 * current_agent_displacement_error_);
    
    // Update current pose with calculated y position
    geometry_msgs::msg::Pose output_virtual_pose_;
    output_virtual_pose_.position.x = default_x;  // Keep x constant
    output_virtual_pose_.position.y = calculated_y;
    output_virtual_pose_.position.z = default_z;  // Keep z constant
    output_virtual_pose_.orientation.x = default_orientation_x;
    output_virtual_pose_.orientation.y = default_orientation_y;
    output_virtual_pose_.orientation.z = default_orientation_z;
    output_virtual_pose_.orientation.w = default_orientation_w;
    
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
    pose_updater_pub_->publish(output_virtual_pose_);
    
    // Optional: Log periodically (every 500 calls = 1 second at 500Hz)
    // static int counter = 0;
    // if (++counter >= 500) {
    //   RCLCPP_INFO(this->get_logger(), 
    //               "Publishing - Mass: %.2f, Damping: %.2f, Stiffness: %.2f, X: %.3f (e=%.3f, u=%.3f)",
    //               mass_value_, damping_value_, calculated_stiffness, calculated_x,
    //               current_agent_displacement_error_, current_position_errors_.x);
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

  // Default pose values
  double default_x = 0.0;
  double default_y = 0.0;
  double default_z = 0.0;
  double default_orientation_x = 0.0;
  double default_orientation_y = 0.0;
  double default_orientation_z = 0.707; // Example value
  double default_orientation_w = 0.707; // Example value
  
  // Storage for most recent messages
  geometry_msgs::msg::Pose current_virtual_pose_;
  float current_force_;
  double current_agent_displacement_error_;
  geometry_msgs::msg::Vector3 current_position_errors_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistributedParameterizer>());
  rclcpp::shutdown();
  return 0;
}