#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class VirtualPosePublisher : public rclcpp::Node {
  public:
    VirtualPosePublisher() : Node("virtual_pose_publisher") {
      pose_publisher = this->create_publisher<geometry_msgs::msg::Pose>(
        "px100_virtual_pose",
        10
      );

      pose_updater_sub = this->create_subscription<geometry_msgs::msg::Pose>(
        "admittance_control/px100_virtual_pose_updater",
        10,
        std::bind(&VirtualPosePublisher::sub_callback, this, _1)
      );

      // position parameters
      this->declare_parameter("x_pos", 0.0);
      this->declare_parameter("y_pos", 0.250048);
      this->declare_parameter("z_pos", 0.098);
      
      // quaternion parameters
      this->declare_parameter("x", 0.0);
      this->declare_parameter("y", 0.0);
      this->declare_parameter("z", 0.707);
      this->declare_parameter("w", 0.707);

      this->declare_parameter("frequency", 1.0);


      this->get_parameter("x_pos", x_pos);
      this->get_parameter("y_pos", y_pos);
      this->get_parameter("z_pos", z_pos);
      
      this->get_parameter("x", x);
      this->get_parameter("y", y);
      this->get_parameter("z", z);
      this->get_parameter("w", w);

      this->get_parameter("frequency", frequency);

      timer = this->create_wall_timer(
        frequency * 1s,
        std::bind(&VirtualPosePublisher::callback, this)
      );
    }

    private:
      double x_pos;
      double y_pos;
      double z_pos;
      double w;
      double x;
      double y;
      double z;
      double frequency;

      rclcpp::TimerBase::SharedPtr timer;
      rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_updater_sub;
      const rclcpp::Logger LOGGER = rclcpp::get_logger("virtual_pose_publisher");

      void callback() {
        auto msg = geometry_msgs::msg::Pose();

        msg.position.x = x_pos;
        msg.position.y = y_pos;
        msg.position.z = z_pos;

        msg.orientation.w = w;
        msg.orientation.x = x;
        msg.orientation.y = y;
        msg.orientation.z = z;
        
        RCLCPP_DEBUG(LOGGER,
          "Publishing: (%f, %f, %f)\n(%f, %f, %f, %f)",
          x_pos,
          y_pos,
          z_pos,
          w,
          x,
          y,
          z
        );

        pose_publisher->publish(msg);
      }

      void sub_callback(const geometry_msgs::msg::Pose &msg) {
        x_pos = msg.position.x;
        y_pos = msg.position.y;
        z_pos = msg.position.z;
        w = msg.orientation.w;
        x = msg.orientation.x;
        y = msg.orientation.y;
        z = msg.orientation.z;
      }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualPosePublisher>());
  rclcpp::shutdown();
  return 0;
}