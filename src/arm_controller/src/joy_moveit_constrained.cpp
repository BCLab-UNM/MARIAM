// C++
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <visualization_msgs/msg/marker.hpp>

// tf
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /**
     * Initializes the move group interface.
     */
    void init_move_group()
    {
      move_group = new MoveGroupInterface(shared_from_this(), "interbotix_arm");
    }
    
    ~JoyMoveitConstrained() 
    {
      delete move_group;
    }

  private:
    MoveGroupInterface *move_group = nullptr;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("joy_moveit_constrained");

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

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
      RCLCPP_INFO(LOGGER, "\n\nReceived a target pose\n\n");
      const std::string pose_ref_frame = move_group->getPoseReferenceFrame();
      const std::string end_effector_link = move_group->getEndEffectorLink();
      auto target_pose = *msg;
      geometry_msgs::msg::Pose curr_pose = get_ee_pose(end_effector_link);

      /* Plane constraints */
      moveit_msgs::msg::PositionConstraint plane_constraint;
      plane_constraint.header.frame_id = pose_ref_frame;
      RCLCPP_INFO(LOGGER, "\n\nPose reference frame: %s\n\n",
                              pose_ref_frame.c_str());
      // sets the link plane_constraint will be applied to
      RCLCPP_INFO(LOGGER, "\n\nEnd-effector link: %s\n\n",
                              end_effector_link.c_str());
      plane_constraint.link_name = end_effector_link;

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = { 0.01, 0.1, 0.2 };
      plane_constraint.constraint_region.primitives.push_back(primitive);

      // creating a pose for the plane_constraint
      // TODO: have the plane position lie in something like "starting_life_pose"
      geometry_msgs::msg::Pose plane_pose;
      plane_pose.position.x = curr_pose.position.x;
      plane_pose.position.y = curr_pose.position.y;
      plane_pose.position.z = curr_pose.position.z;
      
      plane_constraint.constraint_region.primitive_poses.push_back(plane_pose);
      plane_constraint.weight = 1.0;
      this->publish_plane_constraint(plane_pose);

      moveit_msgs::msg::OrientationConstraint orientation_constraint;
      orientation_constraint.header.frame_id = pose_ref_frame;
      orientation_constraint.link_name = end_effector_link;

      orientation_constraint.orientation = curr_pose.orientation;
      orientation_constraint.absolute_x_axis_tolerance = 0.4;
      orientation_constraint.absolute_y_axis_tolerance = 0.4;
      orientation_constraint.absolute_z_axis_tolerance = 0.4;
      orientation_constraint.weight = 1.0;

      moveit_msgs::msg::Constraints constraints;
      constraints.name = "use_equality_constraints";
      constraints.position_constraints.push_back(plane_constraint);
      constraints.orientation_constraints.push_back(orientation_constraint);

      MoveGroupInterface::Plan plan;
      RCLCPP_INFO(LOGGER, "\n\nApplying path constraint\n\n");
      move_group->setPathConstraints(constraints);
      move_group->setPoseTarget(target_pose);
      move_group->setPlanningTime(30);
      bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "\n\nPlan status: %s\n\n", success ? "SUCCESS" : "FAILED");

      if(success)
      {
        RCLCPP_INFO(LOGGER, "\n\nMoving to target pose\n\n");
        move_group->execute(plan);
      }
      else
      {
        RCLCPP_INFO(LOGGER, "\n\nCould not find a plan to move to target pose\n\n");
      }
    }
    
     /**
     * Publishes a visual representation of the constraint.
     */
    void publish_plane_constraint(geometry_msgs::msg::Pose plane_pose)
    {
      RCLCPP_INFO(LOGGER, "Publishing plane");
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.01;
      marker.scale.y = 0.1;
      marker.scale.z = 0.2;
      marker.color.a = 0.5;
      marker.color.r = 0.05;
      marker.color.g = 0.05;
      marker.color.b = 0.05;
      marker.pose = plane_pose;
      marker_pub_->publish(marker);
    }

    geometry_msgs::msg::Pose get_ee_pose(const std::string end_effector)
    {
      geometry_msgs::msg::Pose pose;
      try
      {
        // auto now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped = tf_buffer_->lookupTransform(
          "world", end_effector, rclcpp::Time(0)
        );

        // Extract position
        pose.position.x = transformStamped.transform.translation.x;
        pose.position.y = transformStamped.transform.translation.y;
        pose.position.z = transformStamped.transform.translation.z;

        // Extract orientation
        pose.orientation.x = transformStamped.transform.rotation.x;
        pose.orientation.y = transformStamped.transform.rotation.y;
        pose.orientation.z = transformStamped.transform.rotation.z;
        pose.orientation.w = transformStamped.transform.rotation.w;
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(LOGGER, "Could not transform: %s", ex.what());
      }
      RCLCPP_INFO(LOGGER, "\n\nCurrent end effector position: x=%f, y=%f, z=%f\n\n",
      pose.position.x,pose.position.y,pose.position.z);

      RCLCPP_INFO(LOGGER, "\n\nCurrent end effector quaternion: w=%f, x=%f, y=%f, z=%f\n\n",
      pose.orientation.w,pose.orientation.x,pose.orientation.y,
      pose.orientation.z);
      return pose;
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