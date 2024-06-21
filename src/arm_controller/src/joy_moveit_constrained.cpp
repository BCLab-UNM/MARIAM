// C++
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <visualization_msgs/msg/marker.hpp>

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;

class JoyMoveitConstrained : public rclcpp::Node 
{
  public:
    JoyMoveitConstrained() : Node("joy_moveit_constrained")
    {
      pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "joy_target_pose",
        1,
        std::bind(&JoyMoveitConstrained::target_pose_callback, this, std::placeholders::_1)
      );
      
      marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "plane_constraint",
        10
      );
    }

    /**
     * Initializes the move group interface.
     */
    void init_move_group()
    {
      move_group_interface = new MoveGroupInterface(shared_from_this(), "interbotix_arm");
    }
    
    ~JoyMoveitConstrained() 
    {
      delete move_group_interface;
    }

  private:
    MoveGroupInterface *move_group_interface = nullptr;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;

    /**
     * This callback function is triggered when a pose is published from
     * /joy_target_pose. It creates a constraint in the form of a plane in front
     * of the robot and then tries to develop a plan to move the robot into the
     * target position.
     * 
     * @msg: the target_pose
     */
    void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received a target pose");
      auto target_pose = *msg;

      // creating a PositionConstraint
      moveit_msgs::msg::PositionConstraint plane_constraint;
      plane_constraint.header.frame_id = move_group_interface->getPoseReferenceFrame();
      // sets the link plane_constraint will be applied to
      plane_constraint.link_name = move_group_interface->getEndEffectorLink();
      shape_msgs::msg::SolidPrimitive plane;
      plane.type = shape_msgs::msg::SolidPrimitive::BOX;
      plane.dimensions = { 1.0, 0.0005, 1.0 };
      plane_constraint.constraint_region.primitives.emplace_back(plane);

      // creating a pose for the plane_constraint
      geometry_msgs::msg::Pose plane_pose;
      plane_pose.position.x = target_pose.position.x;
      plane_pose.position.y = target_pose.position.y;
      plane_pose.position.z = target_pose.position.z;
      plane_pose.orientation.x = 0;
      plane_pose.orientation.y = 0;
      plane_pose.orientation.z = target_pose.orientation.z*cos(M_PI_4)
                               + target_pose.orientation.w*sin(M_PI_4);
      plane_pose.orientation.w = target_pose.orientation.w*(M_PI_4)
                               - target_pose.orientation.z*sin(M_PI_4);
      plane_constraint.constraint_region.primitive_poses.emplace_back(plane_pose);
      plane_constraint.weight = 1.0;
      this->publish_plane_constraint(plane_pose);

      moveit_msgs::msg::Constraints plane_constraints;
      plane_constraints.position_constraints.emplace_back(plane_constraint);
      plane_constraints.name = "plane_constraint";

      MoveGroupInterface::Plan plan;
      // move_group_interface->setPathConstraints(plane_constraints);
      move_group_interface->setPoseTarget(target_pose);
      move_group_interface->setPlanningTime(10.0);
      bool success = static_cast<bool> (move_group_interface->plan(plan));

      if(success)
      {
        RCLCPP_INFO(this->get_logger(), "Executing...");
        move_group_interface->execute(plan);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Plan failed");
      }
    }
    
     /**
     * Publishes a visual representation of the constraint.
     */
    void publish_plane_constraint(geometry_msgs::msg::Pose plane_pose)
    {
      RCLCPP_INFO(this->get_logger(), "Publishing plane");
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
      marker_pub_->publish(marker);
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