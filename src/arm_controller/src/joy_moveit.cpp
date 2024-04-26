// CPP Headers
#include <memory>
#include <chrono>
#include <functional>
#include <string>

// ROS Headers
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

// MoveIt Headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

// Placeholders are un speicifed objects "_1" "_3" "_2" etc. that represent parameteres used by the function returned from bind
// A synchronized callback with have multiple placeholders
using std::placeholders::_1;

// Run launch file(s): 
// ros2 run joy joy_node
// ros2 launch arm_controller xsarm_moveit.launch.py robot_model:=px100 robot_name:=monica_arm hardware_type:=actual

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class joy_moveit : public rclcpp::Node
{
  public:
    joy_moveit() : Node("joy_moveit")
    {
      // Manually set MoveIt parameters
      //this->declare_parameter("robot_description", "/home/calvinjs/MARIAM/install/interbotix_xsarm_descriptions/share/interbotix_xsarm_descriptions/urdf/px100.urdf.xacro");
      //this->declare_parameter("robot_description_semantic", "/home/calvinjs/MARIAM/install/interbotix_xsarm_moveit/share/interbotix_xsarm_moveit/config/srdf/px100.srdf.xacro");

      // Set custom home pose
      pose_experiment_home.position.x = 0.0;
      pose_experiment_home.position.y = 0.223;
      pose_experiment_home.position.z = 0.098;
      pose_experiment_home.orientation.x =  0.000;
      pose_experiment_home.orientation.y =  0.000;
      pose_experiment_home.orientation.z =  0.707;
      pose_experiment_home.orientation.w =  0.707;

      // Set path constraint
      ocm.link_name = "px100/ee_arm_link";  // Replace with your actual link name
      ocm.header.frame_id = "px100/base_link";        // The reference frame for the constraint
      ocm.orientation.x =  0.000;
      ocm.orientation.y =  0.000;
      ocm.orientation.z =  0.707;
      ocm.orientation.w =  0.707;                // Specify the desired orientation (as a quaternion)
      ocm.absolute_x_axis_tolerance = 0.3;      // Tolerances for the constraint
      ocm.absolute_y_axis_tolerance = 0.3;
      ocm.absolute_z_axis_tolerance = 0.3;
      ocm.weight = 0.8;                         // The importance of this constraint (1.0 is highest)

      std::string my_namespace = this->get_namespace();
      RCLCPP_INFO(this->get_logger(), "Using namespace: %s'}", my_namespace.c_str());

      // 10 is the depth of the message queue
      joy_subscriber_     = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&joy_moveit::joy_callback, this, std::placeholders::_1));
      goal_pose_joy_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_pose_joy", 10);
    }
    ~joy_moveit() 
    {
      delete move_group_interface;
    }

    void initilize_moveit() {
      // Create a MoveGroupInterface object
      if (move_group_interface == nullptr) move_group_interface = new MoveGroupInterface(shared_from_this(), "interbotix_arm");

      // Create collision object for the robot to avoid
      auto const collision_object_ground = [frame_id = move_group_interface->getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object_ground;
        collision_object_ground.header.frame_id = frame_id;
        collision_object_ground.id = "ground";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1;
        primitive.dimensions[primitive.BOX_Y] = 1;
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.005;

        collision_object_ground.primitives.push_back(primitive);
        collision_object_ground.primitive_poses.push_back(box_pose);
        collision_object_ground.operation = collision_object_ground.ADD;

        return collision_object_ground;
      }();


      // Create collision object the mobile base
      // TODO: Make urdf instead
      auto const collision_object_base = [frame_id = move_group_interface->getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object_base;
        collision_object_base.header.frame_id = frame_id;
        collision_object_base.id = "ground";
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 0.4;
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0;
        box_pose.position.y = -0.17;
        box_pose.position.z = 0.2;
        collision_object_base.primitives.push_back(primitive);
        collision_object_base.primitive_poses.push_back(box_pose);
        collision_object_base.operation = collision_object_base.ADD;
        return collision_object_base;
      }();

      auto const collision_object_wheels = [frame_id = move_group_interface->getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object_wheels;
        collision_object_wheels.header.frame_id = frame_id;
        collision_object_wheels.id = "ground";
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.07;
        primitive.dimensions[primitive.BOX_Y] = 0.25;
        primitive.dimensions[primitive.BOX_Z] = 0.12;
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.235;
        box_pose.position.y = -0.15;
        box_pose.position.z = 0.06;
        collision_object_wheels.primitives.push_back(primitive);
        collision_object_wheels.primitive_poses.push_back(box_pose);
        collision_object_wheels.operation = collision_object_wheels.ADD;
        return collision_object_wheels;
      }();

      // Add the collision object to the scene
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // std::vector<moveit_msgs::msg::CollisionObject> collision_objects{collision_object_ground, collision_object_base, collision_object_wheels};
      // planning_scene_interface.applyCollisionObjects(collision_objects);
      planning_scene_interface.applyCollisionObject(collision_object_base);

    }

  private:

  // member variables
  MoveGroupInterface *move_group_interface = nullptr;
  moveit_msgs::msg::OrientationConstraint ocm;
  bool allow_constraint = false;
  geometry_msgs::msg::Pose pose_goal_initial;
  geometry_msgs::msg::Pose pose_goal;
  std::string pose_goal_named = "None";
  geometry_msgs::msg::Pose pose_experiment_home;
  geometry_msgs::msg::Pose pose_experiment_rest;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pose_joy_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    
    if (msg->axes.size() >= 5) {
      
      // Grab starting goal_pose to see if it changes
      pose_goal_initial = pose_goal;

      // Update pose
      // Z axis is forward and back
      // Y axis inverted up and down
      pose_goal.position.y += -(0.005)*msg->axes[6]; // DPAD left (+) and right (-)
      pose_goal.position.z +=  (0.005)*msg->axes[7]; // DPAD up (+) and down (-)

      // Back button pressed
      if (msg->buttons[6] == 1) {
        pose_goal = pose_experiment_home;
        allow_constraint = false;
      }
      // Home button pressed
      if (msg->buttons[7] == 1) {
        pose_goal_named = "Sleep";
        RCLCPP_INFO(this->get_logger(), "Updated pose to 'Sleep' position"); 
        allow_constraint = false;
      }

      // Print updated pose
      if (pose_goal_initial != pose_goal) {
        RCLCPP_INFO(
          this->get_logger(), 
          "Updated \n Pose: x = %f, y = %f, z = %f \n Quaternion: x = %f, y = %f, z = %f, w = %f", 
          pose_goal.position.x, 
          pose_goal.position.y, 
          pose_goal.position.z,
          pose_goal.orientation.x, 
          pose_goal.orientation.y, 
          pose_goal.orientation.z,
          pose_goal.orientation.w);
      }
      // Publish updated pose (optional)
      // pose_publisher_->publish(pose_goal);

      // Publish goal pose as a marker for rviz
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.03;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.pose = pose_goal;
      goal_pose_joy_publisher_->publish(marker);

      // Attempt execute movement
      if (msg->buttons[2] == 1) { // X button pressed

        // Apply path constraints only if the arm is facing foward
        if (false) {
          moveit_msgs::msg::Constraints test_constraints;
          test_constraints.orientation_constraints.push_back(ocm);
          move_group_interface->setPathConstraints(test_constraints);
        }

        // Example cmd: ros2 topic pub --once /arm_cmd geometry_msgs/msg/Pose "{position: {x: 0.0, y: 0.0, z: 0.2}, orientation: {x: 0.1, y: 0.0, z: 0.0, w: 1.0}}"
        //pose_goal = *msg;
        //RCLCPP_INFO(this->get_logger(), "Received pose {x:'%f', y:'%f', z:'%f'}", msg->position.x, msg->position.y, msg->position.z);
        if(pose_goal_named != "None") {
          move_group_interface->setNamedTarget(pose_goal_named);
        }
        else {
          move_group_interface->setPoseTarget(pose_goal);
        }

        // Create a plan to that target pose
        auto const [success, plan] = [this]{
          moveit::planning_interface::MoveGroupInterface::Plan msg;
          auto const ok = static_cast<bool>(move_group_interface->plan(msg));
          return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if(success) {
          move_group_interface->execute(plan);
          pose_goal_named = "None"; // Reset the initial pose
          RCLCPP_INFO(this->get_logger(), "Planning successful!");

          allow_constraint = true;

        } else {
          RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
      }

      // TODO: Move to initilization so i don't clear it everytime
      // After movement
      move_group_interface->clearPathConstraints();
    }    
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<joy_moveit>();
  node->initilize_moveit(); // can only be done after the constuctor is completed
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
