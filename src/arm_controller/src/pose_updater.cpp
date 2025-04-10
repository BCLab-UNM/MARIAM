#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <thread>


class LiftingNode : public rclcpp::Node {
  private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ross_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr monica_publisher;
  
  public:
    LiftingNode() : Node("pose_updater") {
      ross_publisher = this->create_publisher<geometry_msgs::msg::Pose>(
        "/ross/px100_target_pose",
        50
      );
      
      monica_publisher = this->create_publisher<geometry_msgs::msg::Pose>(
        "/monica/px100_target_pose",
        50
      );
    }

    void update_x_position(double x_pos) {

    }

    void update_z_position(double z_pos) {
      auto msg = geometry_msgs::msg::Pose();
      msg.position.x = 0.0;
      msg.position.y = 0.23;
      msg.position.z = z_pos;

      msg.orientation.w = 0.707;
      msg.orientation.x = 0.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.707;

      ross_publisher->publish(msg);
      monica_publisher->publish(msg);
      RCLCPP_INFO(this->get_logger(), "published %.4f", msg.position.z);
    }
};

int main(int argc, char * argv[]) {
  if (argc < 2) {
    std::cout << "Not enough arguments\n";
    return 1;
  }
  // initial height of the arm (in meters)
  std::string option = std::string(argv[1]);
  double height = 0.067;
  double y_position = 0.23;

  rclcpp::init(argc, argv);
  auto node = LiftingNode();

  if (option == "z") {
    for(int i = 0; i < 50; i++) {
      // increase the height by a millimeter
      height += 0.001;
      // publish a pose every 0.04 seconds
      node.update_z_position(height);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  rclcpp::shutdown();
  return 0;
}