#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"


/**
 * This class contains poses that test the arm's trajectory
 * when a high number of goal poses are published.
 */
class SineWavePublisher : public rclcpp::Node {
  public:
    SineWavePublisher() : Node("sine_wave_publisher") {

      joint_group_pub = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
        "commands/joint_group",
        10
      );
      joint_single_pub = this->create_publisher<interbotix_xs_msgs::msg::JointSingleCommand>(
        "commands/joint_single",
        10
      );

      // put arm in home position
      auto msg = interbotix_xs_msgs::msg::JointGroupCommand();
      msg.name = "arm";
      msg.cmd = {0, 0, 0, 0};
      joint_group_pub->publish(msg);
    }

    void trajectory_trace() {
      RCLCPP_INFO(LOGGER, "Publishing joint commands");

      // wave parameters
      const double WAVE_PERIOD = 2.0; // in seconds
      const double AMP = M_PI_4;

      // loop rate parameters
      const double FREQUENCY = 500; // in Hz
      const auto LOOP_PERIOD = std::chrono::microseconds(static_cast<int>(1e6 / FREQUENCY));

      auto start_time = std::chrono::steady_clock::now();
      auto end_time   = start_time + std::chrono::seconds(120);
      double elapsed_time = 0.0;
      
      while(rclcpp::ok()) {
        auto now = std::chrono::steady_clock::now();

        if (now >= end_time) break;
        
        elapsed_time = std::chrono::duration<double>(now - start_time).count();
        // RCLCPP_INFO(LOGGER, "Elapsed time: %.6f", elapsed_time);
        double cmd_value = AMP * std::sin((2.0 * M_PI / WAVE_PERIOD) * elapsed_time);
        // RCLCPP_INFO(LOGGER, "CMD Value: %.6f", cmd_value);
        // auto msg = interbotix_xs_msgs::msg::JointGroupCommand();
        // msg.name = "arm";
        // msg.cmd = {0.0F, 0.0F, 0.0F, static_cast<float>(cmd_value)};
        // joint_group_pub->publish(msg);

        auto msg = interbotix_xs_msgs::msg::JointSingleCommand();
        msg.name = "wrist_angle";
        msg.cmd = cmd_value;
        joint_single_pub->publish(msg);

        std::this_thread::sleep_until(now + LOOP_PERIOD);
      }
      RCLCPP_INFO(LOGGER, "Done");
    }


  private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr joint_group_pub;
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr joint_single_pub;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("sine_wave_publisher");
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = SineWavePublisher();
  std::this_thread::sleep_for (std::chrono::seconds(2));
  node.trajectory_trace();
  rclcpp::shutdown();
  return 0;
}