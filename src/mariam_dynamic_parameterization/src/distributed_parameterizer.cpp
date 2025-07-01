#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

class DistributedParameterizer : public rclcpp::Node
{
public:
  DistributedParameterizer() : Node("distributed_parameterizer")
  {
    // Initialize hardcoded parameter values
    mass_value_ = 1.0;
    damping_value_ = 10.0;
    stiffness_value_ = 100.0;
    
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
    
    // Create timer for 500 Hz publishing
    timer_ = this->create_wall_timer(
      2ms, // 500 Hz = 1000ms / 500 = 2ms
      std::bind(&DistributedParameterizer::TimerCallback, this));
    
    // Initialize pose message
    current_pose_.position.x = 0.0;
    current_pose_.position.y = 0.0;
    current_pose_.position.z = 0.0;
    current_pose_.orientation.x = 0.0;
    current_pose_.orientation.y = 0.0;
    current_pose_.orientation.z = 0.0;
    current_pose_.orientation.w = 1.0;
    
    current_force_ = 0.0;
    current_agent_displacement_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "DistributedParameterizer node initialized");
    RCLCPP_INFO(this->get_logger(), "Publishing at 500 Hz");
    RCLCPP_INFO(this->get_logger(), "Mass: %.2f, Damping: %.2f, Stiffness: %.2f",
               mass_value_, damping_value_, stiffness_value_);
  }

private:
  void SlamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;  // Note the extra .pose
    
    // Extract covariance matrix (6x6 = 36 elements)
    // Layout: [x, y, z, roll, pitch, yaw] 
    auto covariance = msg->pose.covariance;
    
    // Extract position errors (standard deviations)
    double x_error = sqrt(covariance[0]);   // covariance[0] = x variance
    double y_error = sqrt(covariance[7]);   // covariance[7] = y variance  
    double z_error = sqrt(covariance[14]);  // covariance[14] = z variance
    
    RCLCPP_DEBUG(this->get_logger(), 
                "SLAM pose: [%.3f, %.3f, %.3f], errors: [%.3f, %.3f, %.3f]",
                current_pose_.position.x, current_pose_.position.y, current_pose_.position.z,
                x_error, y_error, z_error);
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
    // Create messages
    auto mass_msg = std_msgs::msg::Float64();
    auto damping_msg = std_msgs::msg::Float64();
    auto stiffness_msg = std_msgs::msg::Float64();
    
    // Update parameters and publish
    // pose_updater_pub_->publish(current_pose_);
    // stiffness_pub_->publish(stiffness_msg);
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
  geometry_msgs::msg::Pose current_pose_;
  float current_force_;
  double current_agent_displacement_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistributedParameterizer>());
  rclcpp::shutdown();
  return 0;
}