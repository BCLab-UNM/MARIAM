#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <vector>
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
//#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class JointDelayTracker : public rclcpp::Node {
  private:
    // this is used to calculate the amount that passes
    // between publishing the joint command and the joint
    // reaching that position
    double desired_joint_position = 0.0;
    // the maximum amount of time to pass before
    // changing reference_joint_command
    double max_diff = 10.0;
    // the time when the reference command was set
    std::chrono::steady_clock::time_point start;

    std::vector<double> time_diffs = {};

    rclcpp::Subscription<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr joint_cmd_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;

  public:
    JointDelayTracker() : Node("joint_delay_tracker") {
      joint_cmd_sub = this->create_subscription<interbotix_xs_msgs::msg::JointSingleCommand>(
        "commands/joint_single",
        10,
        std::bind(&JointDelayTracker::joint_command_callback, this, _1)
      );
      
      joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",
        10,
        std::bind(&JointDelayTracker::joint_states_callback, this, _1)
      );

      start = std::chrono::steady_clock::now();
    }

    void joint_states_callback(const sensor_msgs::msg::JointState &msg) {
      auto now = std::chrono::steady_clock::now();
      auto current_position = msg.position[3];

      if (current_position == desired_joint_position) {
        double time_diff = std::chrono::duration<double>(now - start).count();
        time_diffs.push_back(time_diff);
        RCLCPP_INFO(this->get_logger(), "Reached desired joint position in: %.6f",
          time_diff
        );

        double avg = 0;

        for (double time : time_diffs) {
          avg += time;
        }
        avg /= time_diffs.size();

        RCLCPP_INFO(this->get_logger(), "Current avg.: %.6f", avg);
      }
    }

    /**
     * This function will update the joint angle command
     * to calculate the delay.
     */
    void joint_command_callback(
      const interbotix_xs_msgs::msg::JointSingleCommand &msg
    ) {
      auto cmd = msg.cmd;
      auto now = std::chrono::steady_clock::now();

      if (max_diff <= std::chrono::duration<double>(now - start).count()) {
        //RCLCPP_INFO(this->get_logger(), "Max amount of time has been reached.");
        desired_joint_position = cmd;
        start = now;
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointDelayTracker>());
  rclcpp::shutdown();
  return 0;
}