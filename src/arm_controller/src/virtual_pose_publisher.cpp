#include <chrono>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;

class VirtualPosePublisher : public rclcpp::Node {
  public:
    VirtualPosePublisher() : Node("virtual_pose_publisher") {
      pose_publisher = this->create_publisher<Pose>(
        "high_freq_virtual_pose",
        10
      );
      // position parameters
      this->declare_parameter("x_pos", 0.0);
      this->declare_parameter("y_pos", 0.250048);
      this->declare_parameter("z_pos", 0.098);
      
      // quaternion parameters
      this->declare_parameter("w", 0.707);
      this->declare_parameter("x", 0.0);
      this->declare_parameter("y", 0.0);
      this->declare_parameter("z", 0.707);

      this->declare_parameter("delay",     2.0);
      this->declare_parameter("frequency", 1.0);


      this->get_parameter("x_pos", x_pos);
      this->get_parameter("y_pos", y_pos);
      this->get_parameter("z_pos", z_pos);
      
      this->get_parameter("w", w);
      this->get_parameter("x", x);
      this->get_parameter("y", y);
      this->get_parameter("z", z);
      
      this->get_parameter("delay",     delay);
      this->get_parameter("frequency", frequency);

      delay_timer = this->create_wall_timer(
        delay * 1s,
        std::bind(&VirtualPosePublisher::start_timer, this)
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
      double delay;
      double frequency;
      rclcpp::TimerBase::SharedPtr delay_timer;
      rclcpp::TimerBase::SharedPtr timer;
      rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
      const rclcpp::Logger LOGGER = rclcpp::get_logger("virtual_pose_publisher");

      void start_timer() {
        delay_timer->cancel();
        timer = this->create_wall_timer(
          frequency * 1s,
          std::bind(&VirtualPosePublisher::callback, this)
        );
      }

      void callback() {
        auto msg = Pose();
        msg.position.x = x_pos;
        msg.position.y = y_pos;
        msg.position.z = z_pos;

        msg.orientation.w = w;
        msg.orientation.x = x;
        msg.orientation.y = y;
        msg.orientation.z = z;
        
        RCLCPP_INFO(LOGGER,
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
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualPosePublisher>());
  rclcpp::shutdown();
  return 0;
}