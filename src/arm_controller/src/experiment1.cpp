// CPP Headers
#include <memory>
#include <chrono>
#include <functional>
#include <string>

// ROS Headers
//#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"

// Placeholders are un speicifed objects "_1" "_3" "_2" etc. that represent parameteres used by the function returned from bindg
// A synchronized callback with have multiple placeholders
using std::placeholders::_1;

// Run launch file: ros2 launch arm_controller xsarm_moveit.launch.py robot_model:=px100 robot_name:=monica_arm hardware_type:=actual

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class experiment1: public rclcpp::Node
{
  public:
    experiment1()
    : Node("experiment1")
    {
      // Manually set MoveIt parameters
      //this->declare_parameter("robot_description", "/home/sachib/MARIAM/install/interbotix_xsarm_descriptions/share/interbotix_xsarm_descriptions/urdf/px100.urdf.xacro");
      //this->declare_parameter("robot_description_semantic", "/home/sachib/MARIAM/install/interbotix_xsarm_moveit/share/interbotix_xsarm_moveit/config/srdf/px100.srdf.xacro");

      // 10 is the depth of the message queue
      //subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>("/arm_cmd", 10, std::bind(&experiment1::arm_cmd_callback, this, _1));
      //timer_ = this->create_wall_timer(5s, std::bind(&experiment1::timer_callback, this));
    }
    ~experiment1() 
    {
      //delete move_group_interface;
    }

  private:

  // member variables
  //MoveGroupInterface *move_group_interface = nullptr;
  //geometry_msgs::msg::Pose pose_cmd;
  //rclcpp::TimerBase::SharedPtr timer_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  //rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
  
  void arm_cmd_callback() //const geometry_msgs::msg::Pose::SharedPtr msg
  {
    /*ros::init(argc, argv, "get_end_effector_pose");
    ros::NodeHandle node_handle;

    moveit::planning_interface::MoveGroupInterface move_group_interface("interbotix_arm");
    // Create a MoveGroupInterface once
    // if (move_group_interface == nullptr) move_group_interface = new MoveGroupInterface(shared_from_this(), "interbotix_arm");

    // 
    std::string end_effector_link = "interbotix_gripper";
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    
    geometry_msgs::msg::PoseStamped end_effector_pose;
    end_effector_pose = move_group_interface.getCurrentPose(end_effector_link);

    // Display the end effector pose
    //ROS_INFO_STREAM("End Effector Pose:");
    //ROS_INFO_STREAM("Position: " << end_effector_pose.pose.position);
    //ROS_INFO_STREAM("Orientation: " << end_effector_pose.pose.orientation);

    //pose_cmd = current_pose;
    //pose_cmd.x->(pose_cmd.x + 1);
    //RCLCPP_INFO(this->get_logger(), "Received pose {x:'%f', y:'%f', z:'%f'}", msg->position.x, msg->position.y, msg->position.z);

    //auto move_group_interface = MoveGroupInterface(shared_from_this(), "interbotix_arm");
    
    //move_group_interface->setPoseTarget(*pose_cmd);

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
    */ 
  }
  
  /*void timer_callback()
  {
    // auto message = std_msgs::msg::String();
    // message.data = "MoveIt Interface Node Operating";

    // auto move_group_interface = MoveGroupInterface(shared_from_this(), "interbotix_arm");

    // // geometry_msgs::msg::Pose target_pose;
    // // target_pose.orientation.w = 1.0;
    // // target_pose.position.x = x_cmd;
    // // target_pose.position.y = 0;
    // // target_pose.position.z = 0.23;
    
    // move_group_interface.setPoseTarget(pose_cmd);

    // // Create a plan to that target pose
    // auto const [success, plan] = [&move_group_interface]{
    //   moveit::planning_interface::MoveGroupInterface::Plan msg;
    //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    //   return std::make_pair(ok, msg);
    // }();

    // // Execute the plan
    // if(success) {
    //   move_group_interface.execute(plan);
    //   RCLCPP_ERROR(this->get_logger(), "Planning successful!");
    // } else {
    //   RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    // }
  }
  */
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("experiment1");

  // Initialize MoveIt
  MoveGroupInterface *move_group = new MoveGroupInterface(node, "interbotix_arm");

  // Specify the end effector link
  std::string end_effector_link = "end_effector_link";

  // Get the current robot
  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

  // Get the end effector pose
  geometry_msgs::msg::PoseStamped end_effector_pose;
  end_effector_pose = move_group->getCurrentPose(end_effector_link);

  // Display the end effector pose
  RCLCPP_INFO(node->get_logger(), "End Effector Pose:");
  RCLCPP_INFO(node->get_logger(), "Position: %f %f %f", 
    end_effector_pose.pose.position.x, 
    end_effector_pose.pose.position.y, 
    end_effector_pose.pose.position.z);
  RCLCPP_INFO(node->get_logger(), "Orientation: %f %f %f %f", 
    end_effector_pose.pose.orientation.x, 
    end_effector_pose.pose.orientation.y, 
    end_effector_pose.pose.orientation.z, 
    end_effector_pose.pose.orientation.w);

  rclcpp::shutdown();
  return 0;

/*  rclcpp::init(argc, argv);
  // auto const node = std::make_shared<rclcpp::Node>(
  //   "pos_control_moveit_interface",
  //   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  // );
  //auto move_group_interface = MoveGroupInterface(node, "interbotix_arm");
  //node->move_group_interface = NULL
  //node->move_group_interface = MoveGroupInterface(node, "interbotix_arm");
  //rclcpp::spin(node);
  rclcpp::spin(std::make_shared<_experiment1>());
  rclcpp::shutdown();
  return 0;  */
}
