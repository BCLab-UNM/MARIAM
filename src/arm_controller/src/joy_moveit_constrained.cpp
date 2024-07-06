// C++
#include <memory>
#include <chrono>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "arm_controller/msg/constrained_pose.hpp"
#include "arm_controller/msg/path_and_execution_timing.hpp"

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
using namespace std::chrono;
using namespace arm_controller::msg;
using namespace visualization_msgs::msg;
using namespace geometry_msgs::msg;

/**
 * Topics:
 * - joy_target_pose: this node has a subscription to this topic.
 * Once a target_pose is published to this topic, the node use the
 * moveit move group interface to create a plan and move to the target pose.
 * 
 * - plane_constraint: this node publishes the position and orientation of
 * the plane constraint to this topic, whenever one is created.
 * 
 * - moveit_path_and_execution_timing: this node will publish the time taken
 * to create a plan and execute that plan to this topic.
 */
class JoyMoveitConstrained : public rclcpp::Node 
{
  public:
    JoyMoveitConstrained() : Node("joy_moveit_constrained")
    {
      pose_sub_ = 
          this->create_subscription<ConstrainedPose>(
            "joy_target_pose",
            10,
            std::bind(&JoyMoveitConstrained::targetPoseCallback, this, std::placeholders::_1)
          );
      
      market_pub_ = this->create_publisher<Marker>(
        "plane_constraint",
        10
      );

      timing_pub_ = 
        this->create_publisher<PathAndExecutionTiming>(
          "moveit_path_and_execution_timing",
          10
        );

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = 
              std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /**
     * Initializes the move group interface.
     */
    void init_move_group()
    {
      move_group = 
          new MoveGroupInterface(shared_from_this(), "interbotix_arm");
    }
    
    ~JoyMoveitConstrained() 
    {
      delete move_group;
    }

  private:
    MoveGroupInterface *move_group = nullptr;

    rclcpp::Publisher<Marker>::SharedPtr market_pub_;
    rclcpp::Publisher<PathAndExecutionTiming>::SharedPtr timing_pub_;
    rclcpp::Subscription<ConstrainedPose>::SharedPtr pose_sub_;

    const rclcpp::Logger LOGGER = 
      rclcpp::get_logger("joy_moveit_constrained");

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
    void targetPoseCallback(const ConstrainedPose::SharedPtr msg)
    {
      RCLCPP_INFO(LOGGER, "\n\nReceived a target pose\n\n");
      const std::string pose_ref_frame = move_group->getPoseReferenceFrame();
      const std::string end_effector_link = move_group->getEndEffectorLink();
      auto target_pose = msg->pose;
      moveit_msgs::msg::Constraints constraints;
      
      if(msg->use_plane_constraint)
      {
        Pose curr_pose = getEndEffectorPose(end_effector_link);
        constraints = createConstraints(
          curr_pose,
          pose_ref_frame,
          end_effector_link
        );
        move_group->setPathConstraints(constraints);
      }
      else move_group->clearPathConstraints();

      MoveGroupInterface::Plan plan;
      move_group->setPoseTarget(target_pose);
      move_group->setPlanningTime(30);

      // NOTE: You could also use plan.planning_time_
      auto planning_time_start = high_resolution_clock::now();
      bool success = (
        move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS
      );
      auto planning_time_end = high_resolution_clock::now();
      duration<double> planning_time = duration_cast<duration<double>>(
        planning_time_end - planning_time_start
      );

      if(success)
      {
        auto exectuion_time_start = high_resolution_clock::now();
        move_group->execute(plan);
        auto execution_time_end = high_resolution_clock::now();
        auto execution_time = duration_cast<duration<double>>(
          execution_time_end - exectuion_time_start
        );
        RCLCPP_INFO(LOGGER, "Planning and execution time: %f s, %f s",
          planning_time.count(),
          execution_time.count()
        );
        publishPlanAndExecuteTiming(planning_time.count(),
                                    execution_time.count());
      }
    }
    
    /**
     * Publishes a visual representation of the constraint.
     */
    void publishPlaneConstraint(Pose plane_pose)
    {
      RCLCPP_INFO(LOGGER, "Publishing plane");
      auto marker = Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.id      = 0;
      marker.type    = Marker::CUBE;
      marker.action  = Marker::ADD;
      marker.scale.x = 0.01;
      marker.scale.y = 0.05;
      marker.scale.z = 0.4;
      marker.color.a = 0.5;
      marker.color.r = 0.05;
      marker.color.g = 0.05;
      marker.color.b = 0.05;
      marker.pose    = plane_pose;
      market_pub_->publish(marker);
    }

    void publishPlanAndExecuteTiming(
      double planning_time,
      double execution_time
    )
    {
      auto msg = PathAndExecutionTiming();
      msg.path_planning_time = planning_time;
      msg.execution_time = execution_time;
      timing_pub_->publish(msg);
    }

    /**
     * Creates a plane constraint and an orientation constraint,
     * then adds them to a Constraints object, which 
     * is the object returned by the method.
     */
    moveit_msgs::msg::Constraints createConstraints(
      geometry_msgs::msg::Pose curr_pose,
      const std::string pose_ref_frame,
      const std::string end_effector_link
    )
    {
      RCLCPP_INFO(LOGGER, "\n\nCreating constraints\n\n");
      moveit_msgs::msg::PositionConstraint plane_constraint;
      plane_constraint.header.frame_id = pose_ref_frame;
      plane_constraint.link_name = end_effector_link;

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = { 0.04, 0.08, 0.4 }; // in meters
      plane_constraint.constraint_region.primitives.push_back(primitive);

      Pose plane_pose;
      plane_pose.position.x = curr_pose.position.x;
      plane_pose.position.y = curr_pose.position.y;
      plane_pose.position.z = curr_pose.position.z;
      
      plane_constraint.constraint_region.primitive_poses.push_back(plane_pose);
      plane_constraint.weight = 1.0;
      this->publishPlaneConstraint(plane_pose);

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
      RCLCPP_INFO(LOGGER, "\n\nReturning constraints\n\n");
      return constraints;
    }

    /**
     * This method is used to get the current position of the end effector.
     */
    Pose getEndEffectorPose(const std::string end_effector)
    {
      Pose pose;
      try
      {
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
      RCLCPP_INFO(LOGGER,
        "\n\nCurrent end effector position: x=%f, y=%f, z=%f\n\n",
        pose.position.x,pose.position.y,pose.position.z
      );

      RCLCPP_INFO(LOGGER,
        "\n\nCurrent end effector quaternion: w=%f, x=%f, y=%f, z=%f\n\n",
        pose.orientation.w,pose.orientation.x,pose.orientation.y,
        pose.orientation.z
      );
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