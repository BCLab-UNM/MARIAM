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
// #include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <visualization_msgs/msg/marker.hpp>

using moveit::planning_interface::MoveGroupInterface;

class JoyMoveitConstrained : public rclcpp::Node 
{
  public:
    JoyMoveitConstrained() : rclcpp::Node("joy_moveit_constrained")
    {
      // Set custom home pose
      this->set_custom_home_pose();

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

      joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyMoveitConstrained::joy_callback, this, std::placeholders::_1));
      
      goal_pose_joy_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "goal_pose_joy",
        10
      );

      const_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "box_constraint",
        10
      );
      
    }
    void initilize_moveit() {
      // Create a MoveGroupInterface object
      if (move_group_interface == nullptr) move_group_interface = new MoveGroupInterface(shared_from_this(), "interbotix_arm");

      // Create collision object for the robot to avoid
      auto const collision_object_ground = this->create_collision_ground_obj();

      // Create collision object the mobile base
      // TODO: Make urdf instead
      auto const collision_object_base = this->create_base_collision_obj();

      auto const collision_object_wheels = this->create_wheels_collision_obj();

      // Add the collision object to the scene
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // std::vector<moveit_msgs::msg::CollisionObject> collision_objects{collision_object_ground, collision_object_base, collision_object_wheels};
      // planning_scene_interface.applyCollisionObjects(collision_objects);
      planning_scene_interface.applyCollisionObject(collision_object_base);
    }
    ~JoyMoveitConstrained() 
    {
      delete move_group_interface;
    }

  private:
    // Member variabls
    const rclcpp::Logger LOGGER = this->get_logger();
    MoveGroupInterface *move_group_interface = nullptr;
    moveit_msgs::msg::OrientationConstraint ocm;
    bool allow_constraint = false;
    geometry_msgs::msg::Pose pose_goal_initial;
    geometry_msgs::msg::Pose pose_goal;
    std::string pose_goal_named = "None";
    geometry_msgs::msg::Pose pose_experiment_home;
    geometry_msgs::msg::Pose pose_experiment_rest;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pose_joy_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr const_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    /**
     * This callback function performs specific tasks depending on which
     * button was pressed on the controller.
     * Each button is associate with a specific value, which can be found here: http://wiki.ros.org/joy
     * 
     * @msg: a Joy message containing data on which button was pressed.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      if (msg->axes.size() < 5) return;
      
      // Grab starting goal_pose to see if it changes
      pose_goal_initial = pose_goal;

      // Update pose
      // Z axis is forward and back
      // Y axis inverted up and down
      pose_goal.position.x +=  (0.001)*msg->buttons[3]; // Y
      pose_goal.position.x += -(0.001)*msg->buttons[0]; // A
      pose_goal.position.y += -(0.001)*msg->axes[6]; // DPAD left (+) and right (-)
      pose_goal.position.z +=  (0.001)*msg->axes[7]; // DPAD up (+) and down (-)

      // TODO: add buttons to rotate the base (triggers)

      // Back button pressed
      if (msg->buttons[6] == 1) 
      {
        pose_goal = pose_experiment_home;
        allow_constraint = false;
      }
      // Start button pressed
      if (msg->buttons[7] == 1) 
      {
        pose_goal_named = "Sleep";
        RCLCPP_INFO(this->get_logger(), "Updated pose to 'Sleep' position"); 
        allow_constraint = false;
      }

      // Print updated pose
      if (pose_goal_initial != pose_goal) 
      {
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

      this->publish_goal_pose(pose_goal);

      // Attempt execute movement
      if (msg->buttons[2] == 1) 
      { // X button pressed
        this->attempt_execution();
      }

      // TODO: Move to initilization so i don't clear it everytime
      // After movement
      move_group_interface->clearPathConstraints();
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
      const_pub_->publish(marker);
    }

    /**
     * Publish goal pose as a marker in RViz
     * 
     * @pose_goal:
     */
    void publish_goal_pose(geometry_msgs::msg::Pose pose_goal) 
    {
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
    }

    /**
     * This function attempts to generate a path to the pose_goal.
     * If it is successful, the plan is executed.
     */
    void attempt_execution()
    {
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
      geometry_msgs::msg::Pose plane_pose;
      plane_pose.position.x = pose_goal_initial.position.x;
      plane_pose.position.y = pose_goal_initial.position.y;
      plane_pose.position.z = pose_goal_initial.position.z;
      plane_pose.orientation.x = pose_goal_initial.orientation.x;
      plane_pose.orientation.y = pose_goal_initial.orientation.y;
      plane_pose.orientation.z = pose_goal_initial.orientation.z;
      plane_pose.orientation.w = pose_goal_initial.orientation.w;
      plane_constraint.constraint_region.primitive_poses.emplace_back(plane_pose);
      plane_constraint.weight = 1.0;

      moveit_msgs::msg::Constraints plane_constraints;
      plane_constraints.position_constraints.emplace_back(plane_constraint);
      plane_constraints.name = "use_equality_constraints";
      move_group_interface->setPathConstraints(plane_constraints);
      this->publish_plane_constraint(plane_pose);

      if(pose_goal_named != "None") 
      {
        move_group_interface->setNamedTarget(pose_goal_named);
      }
      else 
      {
        move_group_interface->setPoseTarget(pose_goal);
      }

      // Create a plan to that target pose
      auto const [success, plan] = [this]
      {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_interface->setPoseTarget(pose_goal);
        move_group_interface->setPlanningTime(2.0);
        move_group_interface->plan(plan);
        auto const ok = static_cast<bool>(move_group_interface->plan(plan));
        return std::make_pair(ok, plan);
      }();

      // Execute the plan
      if(success) 
      {
        move_group_interface->execute(plan);
        pose_goal_named = "None"; // Reset the initial pose
        RCLCPP_INFO(this->get_logger(), "Planning successful!");

        allow_constraint = true;

      } 
      else 
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      }
    }
    
    /*=================================================================*/
    /*                            Helper methods                       */
    /*=================================================================*/
    void set_custom_home_pose()
    {
      pose_experiment_home.position.x = 0.0;
      pose_experiment_home.position.y = 0.223;
      pose_experiment_home.position.z = 0.098;
      pose_experiment_home.orientation.x =  0.000;
      pose_experiment_home.orientation.y =  0.000;
      pose_experiment_home.orientation.z =  0.707;
      pose_experiment_home.orientation.w =  0.707;
    }

    moveit_msgs::msg::CollisionObject create_collision_ground_obj()
    {
      auto frame_id = move_group_interface->getPlanningFrame();
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
    }

    moveit_msgs::msg::CollisionObject create_base_collision_obj()
    {
      auto frame_id = move_group_interface->getPlanningFrame();
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
    }

    moveit_msgs::msg::CollisionObject create_wheels_collision_obj()
    {
      auto frame_id = move_group_interface->getPlanningFrame();
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
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyMoveitConstrained>();
  node->initilize_moveit(); // can only be done after the constuctor is completed
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}