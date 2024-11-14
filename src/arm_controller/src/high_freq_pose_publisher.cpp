#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;


/**
 * This class contains poses that test the arm's trajectory
 * when a high number of goal poses are published.
 */
class HighFreqPosePublisher : public rclcpp::Node
{
  public:
    HighFreqPosePublisher() : Node("high_freq_pose_publisher_node")
    {
      pose1.position.x = 0.0;
      pose1.position.y = 0.15;
      pose1.position.z = 0.098;
      pose1.orientation.x =  0.000;
      pose1.orientation.y =  0.000;
      pose1.orientation.z =  0.707;
      pose1.orientation.w =  0.707;
      
      pose2.position.x = 0.0;
      pose2.position.y = 0.244;
      pose2.position.z = 0.098;
      pose2.orientation.x =  0.000;
      pose2.orientation.y =  0.000;
      pose2.orientation.z =  0.707;
      pose2.orientation.w =  0.707;

      pose3.position.x = 0.0;
      pose3.position.y = 0.244;
      pose3.position.z = 0.244;
      pose3.orientation.x =  0.000;
      pose3.orientation.y =  0.000;
      pose3.orientation.z =  0.707;
      pose3.orientation.w =  0.707;

      pose_publisher = this->create_publisher<Pose>(
        "high_freq_target_pose",
        10
      );

      this->declare_parameter("delay",     5.0);
      this->declare_parameter("duration",  300.0);
      this->declare_parameter("frequency", 5.0);
      this->declare_parameter("max_ticks", 500);

      this->get_parameter("delay",     delay);
      this->get_parameter("duration",  duration);
      this->get_parameter("frequency", frequency);
      this->get_parameter("max_ticks", max_ticks);

      srand(42);

      delay_timer = this->create_wall_timer(
        delay * 1s,
        std::bind(&HighFreqPosePublisher::start_timer, this)
      );
    }

  private:
    int pose_num = 1;
    int ticks = 0;
    int max_ticks = 500;
    // poses for testing the arm's motion
    geometry_msgs::msg::Pose pose1;
    geometry_msgs::msg::Pose pose2;
    geometry_msgs::msg::Pose pose3;

    rclcpp::TimerBase::SharedPtr delay_timer;
    rclcpp::TimerBase::SharedPtr main_timer;
    rclcpp::TimerBase::SharedPtr stop_timer;
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("high_freq_pose_publisher_node");

    double delay;
    double duration;
    double frequency;

    void start_timer()
    {
      delay_timer->cancel();

      main_timer = this->create_wall_timer(
        frequency * 1s,
        std::bind(&HighFreqPosePublisher::callback, this)
      );

      stop_timer = this->create_wall_timer(
        duration * 1s, 
        std::bind(&HighFreqPosePublisher::stop, this)
      );
    }

    void callback()
    {
      auto msg = Pose();
      switch (pose_num)
      {
        case 1:
          msg.position = pose1.position;
          msg.orientation = pose1.orientation;
          RCLCPP_INFO(LOGGER, "Publishing pose 1");
          break;
        
        case 2:
          msg.position = pose2.position;
          msg.orientation = pose2.orientation;
          RCLCPP_INFO(LOGGER, "Publishing pose 2");
          break;
        
        case 3:
          msg.position = pose3.position;
          msg.orientation = pose3.orientation;
          RCLCPP_INFO(LOGGER, "Publishing pose 3");
          break;
      }

      pose_publisher->publish(msg);
      ticks++;
      if(ticks == max_ticks)
      {
        ticks = 0;
        pose_num = 1 + (rand() % 3);
      }
    }

    void stop()
    {
      RCLCPP_INFO(this->get_logger(), " ========== Stopping timer ========== ");
      main_timer->cancel();
      stop_timer->cancel();
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HighFreqPosePublisher>());
  rclcpp::shutdown();
  return 0;
}