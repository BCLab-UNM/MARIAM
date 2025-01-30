#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using namespace std_msgs::msg;

/**
 * This class will publish random values to represent the
 * force being applied to the arm.
 */
class ForcePublisher : public rclcpp::Node
{
  public:
    ForcePublisher() : Node("force_publisher") { 
      force_publisher = this->create_publisher<Float64>(
        "force",
        10
      );

      this->declare_parameter("delay", 5.0);
      this->declare_parameter("duration",    300.0);
      this->declare_parameter("frequency",   5.0);
      this->declare_parameter("max_ticks",   500);

      this->get_parameter("delay",     delay);
      this->get_parameter("duration",  duration);
      this->get_parameter("frequency", frequency);
      this->get_parameter("max_ticks", max_ticks);
      
      delay_timer = this->create_wall_timer(
        delay * 1s,
        std::bind(&ForcePublisher::start_timer, this)
      );
    }

  private:
    double current_force = 0;
    int ticks = 0;
    int max_ticks = 500;

    double delay;
    double duration;
    double frequency;
    
    rclcpp::TimerBase::SharedPtr delay_timer;
    rclcpp::TimerBase::SharedPtr main_timer;
    rclcpp::TimerBase::SharedPtr stop_timer;
    rclcpp::Publisher<Float64>::SharedPtr force_publisher;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("force_publisher");

    void start_timer()
    {
      delay_timer->cancel();

      main_timer = this->create_wall_timer(
        frequency * 1s,
        std::bind(&ForcePublisher::callback, this)
      );

      stop_timer = this->create_wall_timer(
        duration * 1s, 
        std::bind(&ForcePublisher::stop, this)
      );
    }

    void callback()
    {
      auto msg = Float64();
      if(ticks == max_ticks)
      {
        ticks = 0;
        if(current_force >= 2) current_force = 0;
        else {
          double increment = (double) rand() / RAND_MAX;
          current_force += increment;
          // current_force = 2;
          RCLCPP_INFO(LOGGER, "Increasing force by %f", increment);
        }
      }
      else ticks++;
      msg.data = current_force;
      RCLCPP_INFO(LOGGER, "Publishing value F = %f", current_force);
      force_publisher->publish(msg);
    }

    void stop()
    {
      RCLCPP_INFO(this->get_logger(), "Canceling timer");
      main_timer->cancel();
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForcePublisher>());
  rclcpp::shutdown();
  return 0;
}