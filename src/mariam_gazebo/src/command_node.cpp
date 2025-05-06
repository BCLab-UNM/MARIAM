
#include "rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

/**
 * This node is used to command the robot(s) in the simulation.
 * Depending on the argument passed when running this node,
 * it will command the robot's wheels or arm.
 */


class CommandNode : public rclcpp::Node {
  private:
    // publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped> cmd_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped> arm_control_pub;

    public:
      CommandNode() {
        // initialize the publishers
        cmd_vel_pub = this->create_publisher(
          "cmd_vel",
          10
        );

        arm_control_pub = this->create_publisher(
          "update_virtual_y_position",
          10
        );
      }

      void logMessage(std::string message) {
        RCLCPP(this->get_logger(), "%s", message);
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
  char* command_arm_forward = "cmd_arm_forward";

  if (std::strcmp(argv[1], command_velocity)){}
  else if(std::strcmp(argv[1], command_arm_up)) {}
  else if(std::strcmp(argv[1], command_arm_forward)) {}
  
  return 0;
}
