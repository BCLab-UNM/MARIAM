#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.hpp>

using std::placeholders::_1;

class JoyPoseUpdate : public rclcpp::Node
{
  public:
    JoyPoseUpdate() : Node("joy_pose_update")
    {
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "target_pose",
        10
      );

      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        std::bind(&JoyPoseUpdate::joy_callback, this, _1)
      );

      // Set custom home pose
      pose_home.position.x = 0.0;
      pose_home.position.y = 0.223;
      pose_home.position.z = 0.098;
      pose_home.orientation.x =  0.000;
      pose_home.orientation.y =  0.000;
      pose_home.orientation.z =  0.707;
      pose_home.orientation.w =  0.707;
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    geometry_msgs::msg::Pose curr_pose;
    geometry_msgs::msg::Pose pose_home;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      geometry_msgs::msg::Pose new_pose = curr_pose;

      // Back button pressed
      if (msg->buttons[6] == 1) {
        RCLCPP_INFO(this->get_logger(), "Publishing home pose");
        pose_publisher_->publish(pose_home);
      }

      // Update pose
      curr_pose.position.x +=  (0.01)*msg->buttons[0];
      curr_pose.position.x += -(0.01)*msg->buttons[3];
      curr_pose.position.y += -(0.01)*msg->axes[6]; // DPAD left (+) and right (-)
      curr_pose.position.z +=  (0.01)*msg->axes[7]; // DPAD up (+) and down (-)

      // Home button pressed
      if (msg->buttons[7] == 1) {
        // RCLCPP_INFO(this->get_logger(), "Updated pose to 'Sleep' position"); 
      }

      // Print updated pose
      if (new_pose != curr_pose) {
        RCLCPP_INFO(
          this->get_logger(), 
          "Updated \n Pose: x = %f, y = %f, z = %f \n Quaternion: x = %f, y = %f, z = %f, w = %f", 
          new_pose.position.x, 
          new_pose.position.y, 
          new_pose.position.z,
          new_pose.orientation.x, 
          new_pose.orientation.y, 
          new_pose.orientation.z,
          new_pose.orientation.w);
      }

      if (msg->buttons[2] == 1) { // X button pressed
        // publish pose
        RCLCPP_INFO(this->get_logger(), "Publishing a target_pose");
        pose_publisher_->publish(new_pose);
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyPoseUpdate>());
  rclcpp::shutdown();
  return 0;
}