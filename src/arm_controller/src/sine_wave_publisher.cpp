#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"


/**
 * This class contains poses that test the arm's trajectory
 * when a high number of goal poses are published.
 */
class SineWavePublisher : public rclcpp::Node {
  public:
    SineWavePublisher() : Node("sine_wave_publisher") {

      publisher = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
        "commands/joint_group",
        10
      );

      // put arm in home position
      auto msg = interbotix_xs_msgs::msg::JointGroupCommand();
      msg.name = "arm";
      msg.cmd = {0, 0, 0, 0};
      publisher->publish(msg);
    }

    void trajectory_trace() {
      RCLCPP_INFO(LOGGER, "Starting trajectory trace");

      const double AMP = M_PI_4;
      const double FREQUENCY = 10; // in Hz

      auto start_time = std::chrono::steady_clock::now();
      auto end_time   = start_time + std::chrono::seconds(120);
      double elapsed_time = 0.0;
      
      while(rclcpp::ok()) {
        auto now = std::chrono::steady_clock::now();

        if (now >= end_time) break;
        
        elapsed_time = std::chrono::duration<double>(now - start_time).count();

        double cmd_value = AMP * std::sin((2.0 * M_PI / FREQUENCY) * elapsed_time);
        auto msg = interbotix_xs_msgs::msg::JointGroupCommand();
        msg.name = "arm";
        msg.cmd = {0.0F, 0.0F, 0.0F, static_cast<float>(cmd_value)};
        publisher->publish(msg);
      }
      RCLCPP_INFO(LOGGER, "Ending trajectory trace");
    }


  private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr publisher;
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