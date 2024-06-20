// C++
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <visualization_msgs/msg/marker.hpp>

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;

class JoyMoveitConstrained : public rclcpp::Node 
{
  public:
    JoyMoveitConstrained() : Node("joy_moveit_constrained")
    {
      joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyMoveitConstrained::joy_callback, this, std::placeholders::_1));
      
      // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      //   "plane_constraint",
      //   10
      // );
    }
    void init_move_group()
    {
      move_group_interface = new MoveGroupInterface(shared_from_this(), "interbotix_arm");
    }
    ~JoyMoveitConstrained() 
    {
      delete move_group_interface;
    }

  private:
    // Member variabls
    MoveGroupInterface *move_group_interface = nullptr;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    /**
     * Applies a plane constraint to the gripper.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received message from joy topic\nMsg size: %ld",msg->axes.size());
      auto curr_pose = move_group_interface->getCurrentPose();

      // generating a box constraint
      moveit_msgs::msg::PositionConstraint plane_constraint;
      plane_constraint.header.frame_id = move_group_interface->getPoseReferenceFrame();
      plane_constraint.link_name = move_group_interface->getEndEffectorLink();
      shape_msgs::msg::SolidPrimitive plane;
      plane.type = shape_msgs::msg::SolidPrimitive::BOX;
      plane.dimensions = { 1.0, 0.0005, 1.0 };
      plane_constraint.constraint_region.primitives.emplace_back(plane);
      /* TODO: write code to detect what direction the robot is facing,
      otherwise the box won't always form a proper plane perpendicular to the arm */

      // generating position of plane
      // plane must be parallel to y axis
      geometry_msgs::msg::Pose plane_pose;
      plane_pose.position.x = curr_pose.pose.position.x;
      plane_pose.position.y = curr_pose.pose.position.y;
      plane_pose.position.z = curr_pose.pose.position.z;
      plane_pose.orientation.x = curr_pose.pose.orientation.x;
      plane_pose.orientation.y = curr_pose.pose.orientation.y;
      plane_pose.orientation.z = curr_pose.pose.orientation.z;
      plane_pose.orientation.w = curr_pose.pose.orientation.w;
      plane_constraint.constraint_region.primitive_poses.emplace_back(plane_pose);
      plane_constraint.weight = 1.0;

      moveit_msgs::msg::Constraints plane_constraints;
      plane_constraints.position_constraints.emplace_back(plane_constraint);
      plane_constraints.name = "plane_constraint";
      move_group_interface->setPathConstraints(plane_constraints);
      this->publish_plane_constraint(plane_pose);
    }
    
     /**
     * Publishes a visual representation of the constraint.
     */
    void publish_plane_constraint(geometry_msgs::msg::Pose plane_pose)
    {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.5;
      marker.scale.y = 0.01;
      marker.scale.z = 0.5;
      marker.color.a = 0.5;
      marker.color.r = 0.05;
      marker.color.g = 0.05;
      marker.color.b = 0.05;
      marker.pose = plane_pose;
      // marker_pub_->publish(marker);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyMoveitConstrained>();
  node->init_move_group();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}