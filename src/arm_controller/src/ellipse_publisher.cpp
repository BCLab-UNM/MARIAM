#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;

class EllipsePublisher : public rclcpp::Node
{
  public:
    EllipsePublisher() : rclcpp::Node("ellipse_publisher_node") {
      pose_publisher = this->create_publisher<Pose>(
        "px100_target_pose",
        10
      );
      this->declare_parameter("delay",     2.0);
      this->declare_parameter("duration",  300.0);
      this->declare_parameter("frequency", 1.0);
      this->declare_parameter("max_ticks", 500);

      this->get_parameter("delay",     delay);
      this->get_parameter("duration",  duration);
      this->get_parameter("frequency", frequency);
      this->get_parameter("max_ticks", max_ticks);
      
      waist_angle = compute_waist_position();

      delay_timer = this->create_wall_timer(
        delay * 1s,
        std::bind(&EllipsePublisher::start_timer, this)
      );
    }

  private:
    // The position on the ellipse
    double theta = 0;
    double theta_step = M_PI / 256.0;
    // angle the waist needs to be in (basically yaw angle)
    double waist_angle;
    int ticks = 0;
    int max_ticks = 500;

    double delay;
    double duration;
    double frequency;
    
    rclcpp::TimerBase::SharedPtr delay_timer;
    rclcpp::TimerBase::SharedPtr main_timer;
    rclcpp::TimerBase::SharedPtr stop_timer;
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("ellipse_publisher_node");

    void start_timer()
    {
      delay_timer->cancel();

      main_timer = this->create_wall_timer(
        frequency * 1s,
        std::bind(&EllipsePublisher::callback, this)
      );

      stop_timer = this->create_wall_timer(
        duration * 1s, 
        std::bind(&EllipsePublisher::stop, this)
      );
    }

    void callback()
    {
      // calculating next pose
      auto msg = Pose();
      msg.position.x = (1.0/8.0) * cos(theta);
      msg.position.y = 0.223;
      msg.position.z = (1.0/16.0) * sin(theta) + 0.1605;
      msg.orientation.x =  0.0;
      msg.orientation.y =  0.0;
      msg.orientation.z =  sin(waist_angle/2);
      msg.orientation.w =  cos(waist_angle/2);
      RCLCPP_INFO(LOGGER,
        "Sending pose with angles (%f, %f)", 
        theta, waist_angle);
      RCLCPP_INFO(LOGGER,
                  "Position:\n<%f, %f, %f>",
                  msg.position.x,
                  msg.position.y,
                  msg.position.z);
      RCLCPP_INFO(LOGGER,
                  "Quaternion:\n<%f, %f, %f, %f>",
                  msg.orientation.x,
                  msg.orientation.y,
                  msg.orientation.z,
                  msg.orientation.w);
      pose_publisher->publish(msg);

      if (ticks == max_ticks)
      {
        // RCLCPP_INFO(LOGGER, "Reached max ticks");
        ticks = 0;
        if (theta >= 2 * M_PI) theta = 0;
        else theta += theta_step;
        waist_angle = compute_waist_position();
      }
      else ticks++;
    }

    void stop()
    {
      RCLCPP_INFO(this->get_logger(), "Canceling timer");
      main_timer->cancel();
    }

    double compute_waist_position() 
    {
      double num = (1.0/8.0) * std::cos(theta);
      double dem = std::sqrt((1.0/64.0) * std::cos(theta) + (0.223*0.223));
      return std::acos((double) num / dem);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EllipsePublisher>());
  rclcpp::shutdown();
  return 0;
}