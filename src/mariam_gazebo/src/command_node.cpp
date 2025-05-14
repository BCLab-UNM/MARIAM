#include "rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// NOTE: should switch to float 64 in the future?
#include "std_msgs/msg/float32.hpp"

/**
 * This node is used to command the robot(s) in the simulation.
 * Depending on the argument passed when running this node,
 * it will command the robot's wheels or arm.
 */


class CommandNode : public rclcpp::Node {
  private:
    // publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped> cmd_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float32> arm_control_pub;

  public:
    CommandNode() {
      // initialize the publishers
      cmd_vel_pub = this->create_publisher(
        "cmd_vel",
        10
      );

      arm_control_pub = this->create_publisher(
        "px100_virtual_pose_updater",
        10
      );
    }

    void update_virtual_z_position(float z) {
      auto msg = std_msgs::msg::Float32();
      msg.data = z;
      arm_control_pub.publish(msg);
    }

    void cmd_vel(double x) {
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = x;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      cmd_vel_pub.publish(msgs);
    }
}


int main(int argc, char* argv[]) {
  // if no command was given, exit
  if (argc < 1) {
    return 1;
  }

  rclcpp::init(argc, argv);

  auto command_node = CommandNode();
  char* command_velocity = "cmd_vel";
  char* command_arm_up = "cmd_arm_up";

  if(std::strcmp(argv[1], command_arm_up)) {
    int height = 0.067;
    for(int i = 0; i < 50; i++) {
      // increase the height by a millimeter
      height += 0.001;
      // publish a pose every 0.04 seconds
      command_node.arm_control_pub(height);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  else if(std::strcmp(argv[1], command_arm_forward)) {
    // command the robot to move one meter
    // TODO: should be +1 meter for the other robot
    command_node.cmd_vel(-1.0);
  }
  
  return 0;
}
