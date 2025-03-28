#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <thread>

class LiftingNode : public rclcpp::Node {
  private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr virtual_z_pos_pub;
  
  public:
    LiftingNode() : Node("lifting_node") {
      virtual_z_pos_pub = this->create_publisher<std_msgs::msg::Float64>(
        "px100_virtual_pose_updater",
        10
      );
    }

    void publish_z_position(double z_pos) {
      auto msg = std_msgs::msg::Float64();
      msg.data = z_pos;
      virtual_z_pos_pub->publish(msg);
      RCLCPP_INFO(this->get_logger(), "published %.4f", msg.data);
    }
};

int main(int argc, char * argv[]) {
  // initial height of the arm (in meters)
  double height = 0.067;
  rclcpp::init(argc, argv);
  auto node = LiftingNode();

  for(int i = 0; i < 5; i++) {
    // increase the height by a centimeter
    height += 0.01;
    // publish a pose every 0.4 seconds
    node.publish_z_position(height);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
  }

  rclcpp::shutdown();
  return 0;
}