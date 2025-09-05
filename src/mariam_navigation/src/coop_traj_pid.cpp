#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

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
  }

private:
  void control_loop(const geometry_msgs::msg::Pose::SharedPtr msg) {}

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base2_sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoopTrajPID>());
  rclcpp::shutdown();
  return 0;
}
