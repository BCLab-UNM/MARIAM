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
          "update_virtual_y_position",
          10
        );
      }

      void publish_new_y_position(float y) {
        auto msg = std_msgs::msg::Float32();
        msg.data = 
      }

      void publish_new_z_position(float z) {
        
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
  else if(std::strcmp(argv[1], command_arm_up)) {

  }
  else if(std::strcmp(argv[1], command_arm_forward)) {}
  
  return 0;
}
