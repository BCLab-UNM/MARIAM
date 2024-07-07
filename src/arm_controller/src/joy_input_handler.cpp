#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "arm_controller/msg/constrained_pose.hpp"

using std::placeholders::_1;
using namespace arm_controller::msg;
using namespace visualization_msgs::msg;
using namespace geometry_msgs::msg;

class JoyInputHandler : public rclcpp::Node
{
  public:
    JoyInputHandler() : Node("joy_input_handler")
    {
      pose_publisher_ = this->create_publisher<ConstrainedPose>(
        "joy_target_pose",
        10
      );

      goal_pose_joy_publisher_ = this->create_publisher<Marker>(
        "goal_pose_joy",
        10
      );

      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        std::bind(&JoyInputHandler::joy_callback, this, _1)
      );

      pose_test_start.position.x = 0.250048;
      pose_test_start.position.y = 0;
      pose_test_start.position.z = 0.098;
      pose_test_start.orientation.x =  0.000;
      pose_test_start.orientation.y =  0.000;
      pose_test_start.orientation.z =  0.000;
      pose_test_start.orientation.w =  1.000;

      pose_test_end.position.x = 0.250048;
      pose_test_end.position.y = 0;
      pose_test_end.position.z = 0.244;
      pose_test_end.orientation.x =  0.000;
      pose_test_end.orientation.y =  0.000;
      pose_test_end.orientation.z =  0.000;
      pose_test_end.orientation.w =  1.000;
    }

  private:
    rclcpp::Publisher<ConstrainedPose>::SharedPtr
      pose_publisher_;
    rclcpp::Publisher<Marker>::SharedPtr
      goal_pose_joy_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    Pose curr_pose;
    // poses for testing the arm's motion
    Pose pose_test_start;
    Pose pose_test_end;
    float theta = 0;
    bool safe_to_publish = true;
    bool apply_constraints = false;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("joy_input_handler");

    /**
     * This method is the callback function for /joy.
     * It reads the message from /joy and performs specific actions
     * depending on which button was pressed.
     * xbox360 layout can be found here: http://wiki.ros.org/joy
     * 
     * Controls:
     * Left stick to move the target pose in the y and z directions.
     * Right stick to move the target pose in the x direction.
     * 'x' to publish the target pose.
     * 'Back' to publish the Sleep pose.
     * 'Start' to publish the Home pose.
     * 'LB' & 'RB' to publish a custom pose.
     * 'Y' to apply path constraints.
     * 
     * @msg: controller input
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      Pose new_pose = curr_pose;

      // LB
      if (msg->buttons[4] == 1 && safe_to_publish) 
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing pose_test_start\n\n");
        curr_pose = pose_test_start;
        this->publish_marker(pose_test_start);
        this->publish_pose(pose_test_start, true, "none");
        safe_to_publish = false;
      }

      // RB
      if(msg->buttons[5] == 1 && safe_to_publish)
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing pose_test_end\n\n");
        curr_pose = pose_test_end;
        this->publish_marker(pose_test_end);
        this->publish_pose(new_pose, true, "none");
        safe_to_publish = false;
      }

      // Back
      if(msg->buttons[6] == 1)
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing Sleep pose\n\n");
        curr_pose = pose_test_end;
        this->publish_pose(new_pose, false, "Sleep");
        safe_to_publish = false;
      }

      // Start
      if(msg->buttons[7] == 1)
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing Home pose\n\n");
        curr_pose = pose_test_end;
        this->publish_pose(new_pose, false, "Home");
        safe_to_publish = false;
      }
      
      // X: publish pose
      if (msg->buttons[2] && safe_to_publish)
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing target pose\n\n");
        this->publish_pose(new_pose, apply_constraints, "none");
        safe_to_publish = false;
      }

      // Update pose
      curr_pose.position.x +=  (0.001)*msg->axes[3]; // right stick
      curr_pose.position.x +=  (0.001)*msg->axes[4]; // right stick
      curr_pose.position.y += -(0.001)*msg->axes[0]; // left stick
      curr_pose.position.z +=  (0.001)*msg->axes[1]; // left stick

      // LT: rotate counter clockwise
      if(msg->axes[2] == 1)
      {
        theta += (0.05)*msg->axes[2];
        curr_pose.orientation.w = cos(theta);
        curr_pose.orientation.z = sin(theta);
      }
      
      // RT: rotate clockwise
      if(msg->axes[5] == 1)
      {
        theta -= (0.05)*msg->axes[5];
        curr_pose.orientation.w = cos(theta);
        curr_pose.orientation.z = sin(theta);
      }

      // RT change value for applying constraints
      if(msg->buttons[3])
      {
        apply_constraints = !apply_constraints;
        RCLCPP_INFO(LOGGER,
          "Apply constraints value: %d",
          apply_constraints
        );
      }

      // Print updated pose
      if (new_pose != curr_pose) 
      {
        safe_to_publish = true;
        RCLCPP_INFO(
          LOGGER,
          "Position: x = %f, y = %f, z = %f \n Quaternion: x = %f, y = %f, z = %f, w = %f", 
          new_pose.position.x, 
          new_pose.position.y, 
          new_pose.position.z,
          new_pose.orientation.x, 
          new_pose.orientation.y, 
          new_pose.orientation.z,
          new_pose.orientation.w);
        this->publish_marker(new_pose);
      }
    }

    void publish_pose(Pose new_pose, bool use_constraints, std::string pose_name)
    {
      auto msg = ConstrainedPose();
      msg.pose = new_pose;
      msg.use_plane_constraint = use_constraints;
      msg.pose_name = pose_name;
      pose_publisher_->publish(msg);
    }

    void publish_marker(Pose new_pose)
    {
      // Publish goal pose as a marker for rviz
      auto marker = Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.id      = 0;
      marker.type    = Marker::ARROW;
      marker.action  = Marker::ADD;
      marker.scale.x = 0.03;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.pose    = new_pose;
      goal_pose_joy_publisher_->publish(marker);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyInputHandler>());
  rclcpp::shutdown();
  return 0;
}