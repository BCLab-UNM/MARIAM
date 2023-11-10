#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// Run launch file: ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=wx250 hardware_type:=fake

// Planning execution rejected

/*
int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pos_control_moveit_interface",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pos_control_moveit_interface");
  RCLCPP_INFO(logger, "Logger created");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "interbotix_arm");
  RCLCPP_INFO(logger, "MoveGroupInterface created!");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 0.23;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
    RCLCPP_ERROR(logger, "Planning successful!");
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  //rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/

#include <chrono>
#include <functional>
#include <string>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class pos_control_moveit_interface : public rclcpp::Node
{
  public:
    pos_control_moveit_interface()
    : Node("pos_control_moveit_interface"), count_(0)
    {
      // Manually set MoveIt parameters
      //this->declare_parameter("robot_description", "/home/calvinjs/MARIAM/install/interbotix_xsarm_descriptions/share/interbotix_xsarm_descriptions/urdf/px100.urdf.xacro");
      //this->declare_parameter("robot_description_semantic", "/home/calvinjs/MARIAM/install/interbotix_xsarm_moveit/share/interbotix_xsarm_moveit/config/srdf/px100.srdf.xacro");
      
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(5s, std::bind(&pos_control_moveit_interface::timer_callback, this));
      //move_group_interface = new MoveGroupInterface(*this, "interbotix_arm");
    }
    //MoveGroupInterface *move_group_interface;

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "MoveIt Interface Node Operating" + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);

      auto move_group_interface = MoveGroupInterface(shared_from_this(), "interbotix_arm");

      // Set a target Pose
      auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0;
        msg.position.y = 0;
        msg.position.z = 0.23;
        return msg;
      }();
      move_group_interface.setPoseTarget(target_pose);

      // Create a plan to that target pose
      auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();

      // Execute the plan
      if(success) {
        move_group_interface.execute(plan);
        RCLCPP_ERROR(this->get_logger(), "Planning successful!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // auto const node = std::make_shared<rclcpp::Node>(
  //   "pos_control_moveit_interface",
  //   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  // );
  //auto move_group_interface = MoveGroupInterface(node, "interbotix_arm");
  //node->move_group_interface = NULL
  //node->move_group_interface = MoveGroupInterface(node, "interbotix_arm");
  //rclcpp::spin(node);
  rclcpp::spin(std::make_shared<pos_control_moveit_interface>());
  rclcpp::shutdown();
  return 0;
}