// CPP Headers
#include <memory>
#include <chrono>
#include <functional>
#include <string>

// ROS Headers
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"

// Placeholders are un speicifed objects "_1" "_3" "_2" etc. that represent parameteres used by the function returned from bind
// A synchronized callback with have multiple placeholders
using std::placeholders::_1;

// Run launch file: ros2 launch arm_controller xsarm_moveit.launch.py robot_model:=px100 robot_name:=monica_arm hardware_type:=actual

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class pos_control_moveit_interface : public rclcpp::Node
{
  public:
    pos_control_moveit_interface()
    : Node("pos_control_moveit_interface")
    {
      // Manually set MoveIt parameters
      //this->declare_parameter("robot_description", "/home/calvinjs/MARIAM/install/interbotix_xsarm_descriptions/share/interbotix_xsarm_descriptions/urdf/px100.urdf.xacro");
      //this->declare_parameter("robot_description_semantic", "/home/calvinjs/MARIAM/install/interbotix_xsarm_moveit/share/interbotix_xsarm_moveit/config/srdf/px100.srdf.xacro");

      std::string my_namespace = this->get_namespace();
      RCLCPP_INFO(this->get_logger(), "Using namespace: %s'}", my_namespace.c_str());

      // 10 is the depth of the message queue
      subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>("arm_cmd", 10, std::bind(&pos_control_moveit_interface::arm_cmd_callback, this, _1));
    }
    ~pos_control_moveit_interface() 
    {
      delete move_group_interface;
    }

  private:

  // member variables
  MoveGroupInterface *move_group_interface = nullptr;
  geometry_msgs::msg::Pose pose_cmd;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
  
  void arm_cmd_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // Create a MoveGroupInterface once
    if (move_group_interface == nullptr) move_group_interface = new MoveGroupInterface(shared_from_this(), "interbotix_arm");

    // Example cmd: ros2 topic pub --once /arm_cmd geometry_msgs/msg/Pose "{position: {x: 0.0, y: 0.0, z: 0.2}, orientation: {x: 0.1, y: 0.0, z: 0.0, w: 1.0}}"
    pose_cmd = *msg;
    RCLCPP_INFO(this->get_logger(), "Received pose {x:'%f', y:'%f', z:'%f'}", msg->position.x, msg->position.y, msg->position.z);
    move_group_interface->setPoseTarget(*msg);

    // Create a plan to that target pose
    auto const [success, plan] = [this]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface->plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
      move_group_interface->execute(plan);
      RCLCPP_ERROR(this->get_logger(), "Planning successful!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pos_control_moveit_interface>());
  rclcpp::shutdown();
  return 0;
}
