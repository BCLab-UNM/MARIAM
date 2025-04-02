#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <thread>


class LiftingNode : public rclcpp::Node {
  private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr virtual_z_pos_pub_ross;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr virtual_z_pos_pub_monica;
  
  public:
    LiftingNode() : Node("lifting_node") {
      virtual_z_pos_pub_ross = this->create_publisher<std_msgs::msg::Float64>(
        "/ross/px100_virtual_pose_updater",
        50
      );
      
      virtual_z_pos_pub_monica = this->create_publisher<std_msgs::msg::Float64>(
        "/monica/px100_virtual_pose_updater",
        50
      );
    }

    void publish_z_position(double z_pos) {
      auto msg = std_msgs::msg::Float64();
      msg.data = z_pos;
      virtual_z_pos_pub_ross->publish(msg);
      virtual_z_pos_pub_monica->publish(msg);
      RCLCPP_INFO(this->get_logger(), "published %.4f", msg.data);
    }
};

int main(int argc, char * argv[]) {
  // initial height of the arm (in meters)
  double height = 0.067;
  rclcpp::init(argc, argv);
  auto node = LiftingNode();

  for(int i = 0; i < 50; i++) {
    // increase the height by a millimeter
    height += 0.001;
    // publish a pose every 0.04 seconds
    node.publish_z_position(height);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  rclcpp::shutdown();
  return 0;
}