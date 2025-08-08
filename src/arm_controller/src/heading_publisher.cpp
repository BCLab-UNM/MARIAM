#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class ArmHeadingPublisher : public rclcpp::Node
{
public:
  ArmHeadingPublisher() : Node("manipulator_heading_publisher") {
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a publisher for the manipulator's heading
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Pose>(
        "px100_base_link_pose",
        1
      );

    timer_ = this->create_wall_timer(
      10ms,
      std::bind(&ArmHeadingPublisher::on_timer, this));
  }

private:
  void on_timer()
  {
    // TODO: change this to the correct frame
    std::string referenceFrame = "world";
    std::string baseLinkFrame = "px100/px100_base_link";

    // get the transform of the target frame in the reference frame
    geometry_msgs::msg::TransformStamped t;

    try {
      t = tf_buffer_->lookupTransform(
        referenceFrame, baseLinkFrame,
        tf2::TimePointZero);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not get transform %s in %s: %s",
        baseLinkFrame.c_str(), referenceFrame.c_str(), ex.what());
      return;
    }

    auto msg = geometry_msgs::msg::Pose();
    msg.position.x = t.transform.translation.x;
    msg.position.y = t.transform.translation.y;
    msg.position.z = t.transform.translation.z;
    msg.orientation.x = t.transform.rotation.x;
    msg.orientation.y = t.transform.rotation.y;
    msg.orientation.z = t.transform.rotation.z;
    msg.orientation.w = t.transform.rotation.w;

    RCLCPP_DEBUG(
      this->get_logger(), "Publishing manipulator heading w.r.t frame: %s",
      referenceFrame.c_str()
    );
    
    RCLCPP_DEBUG(
      this->get_logger(), "Publishing manipulator position: "
      "x: %f, y: %f, z: %f",
      msg.position.x, msg.position.y, msg.position.z);
    
    RCLCPP_DEBUG(
      this->get_logger(), "Publishing manipulator heading: "
      "x: %f, y: %f, z: %f, w: %f",
      msg.orientation.x, msg.orientation.y,
      msg.orientation.z, msg.orientation.w);
    
    

    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmHeadingPublisher>());
  rclcpp::shutdown();
  return 0;
}