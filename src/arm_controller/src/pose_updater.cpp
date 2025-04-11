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

    /**
     * This function will publish a new pose to /px100_target_pose for
     * both robots.
     * 
     * @param y_pos: the new y position
     * @param z_pos: the new z position
     */
    void update_position(double y_pos, double z_pos) {
      auto msg = geometry_msgs::msg::Pose();
      msg.position.x = 0.0;
      msg.position.y = y_pos;
      msg.position.z = z_pos;

      msg.orientation.w = 0.707;
      msg.orientation.x = 0.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.707;

      ross_publisher->publish(msg);
      monica_publisher->publish(msg);
      RCLCPP_DEBUG(this->get_logger(), "published %.4f", msg.position.z);
    }
};

int main(int argc, char * argv[]) {
  if (argc < 2) {
    std::cout << "Not enough arguments\n";
    std::cout << "Please use 'lift' or 'squeeze'\n";
    return 1;
  }
  // initial height of the arm (in meters)
  std::string option = std::string(argv[1]);

  // all positions are in meters
  double initial_y_position = 0.23;
  // the position of the arms when squeezing the object
  double final_y_position = 
                    initial_y_position + 0.05;
  double height = 0.067; 

  rclcpp::init(argc, argv);
  auto node = LiftingNode();

  if (option == "squeeze") {
    for(int i = 0; i < 50; i++) {
      // increase y_position by a millimeter
      initial_y_position += 0.001;
      // update the pose every 0.015 seconds
      node.update_position(initial_y_position, height);
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
  }

  if (option == "lift") {
    for(int i = 0; i < 100; i++) {
      // increase the height by a millimeter
      height += 0.001;
      // update the pose every 0.015 seconds
      node.update_position(0.280, height);
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
  }

  rclcpp::shutdown();
  return 0;
}