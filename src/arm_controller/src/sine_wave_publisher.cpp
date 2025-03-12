#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;


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

      // start a timer to call "timer_callback"
      timer = this->create_wall_timer(
        frequency * 1s,
        std::bind(&SineWavePublisher::timer_callback, this)
      );
    }


  private:
    int count = 0;
    double frequency = 0.2; // 500Hz
    double complete_cycle = 3; // 3 seconds
    double step = 2 * M_PI / (frequency * complete_cycle);

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr publisher;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("sine_wave_publisher");

    void timer_callback() {
      float cmd = M_PI_2 * sin(step * count);
      auto msg = interbotix_xs_msgs::msg::JointGroupCommand();
      msg.name = "arm"; // specifies we want to move the arm
      // creating the joint commands as a list
      msg.cmd = {cmd, cmd, cmd, cmd};

      publisher->publish(msg);
      count++;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SineWavePublisher>());
  rclcpp::shutdown();
  return 0;
}