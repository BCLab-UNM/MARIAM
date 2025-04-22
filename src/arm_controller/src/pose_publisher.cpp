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
class PosePublisher : public rclcpp::Node
{
  public:
    PosePublisher() : Node("pose_publisher_node")
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

      this->declare_parameter("delay",     2.0);
      this->declare_parameter("duration",  300.0);
      this->declare_parameter("frequency", 0.002);
      this->declare_parameter("max_ticks", 500);

      this->get_parameter("delay",     delay);
      this->get_parameter("duration",  duration);
      this->get_parameter("frequency", frequency);
      this->get_parameter("max_ticks", max_ticks);

      pose_publisher = this->create_publisher<Pose>(
        "px100_target_pose",
        10
      );

      // srand(42);

      delay_timer = this->create_wall_timer(
        delay * 1s,
        std::bind(&PosePublisher::start_timer, this)
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
    geometry_msgs::msg::Pose current_pose;

    rclcpp::TimerBase::SharedPtr delay_timer;
    rclcpp::TimerBase::SharedPtr main_timer;
    rclcpp::TimerBase::SharedPtr stop_timer;
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_publisher_node");

    double delay;
    double duration;
    double frequency;

    void start_timer()
    {
      delay_timer->cancel();
      RCLCPP_INFO(LOGGER, "Publishing poses to topic: %s/px100_target_pose",
        this->get_namespace());
      current_pose = pose1;
      main_timer = this->create_wall_timer(
        frequency * 1s,
        std::bind(&PosePublisher::callback, this)
      );

      stop_timer = this->create_wall_timer(
        duration * 1s, 
        std::bind(&PosePublisher::stop, this)
      );
    }

    void callback()
    {
      auto msg = Pose();
      msg.position = current_pose.position;
      msg.orientation = current_pose.orientation;
      pose_publisher->publish(msg);
      ticks++;
      if(ticks == max_ticks)
      {
        ticks = 0;
        pose_num = (pose_num + 1) % 3;
        switch(pose_num){
          case 1:
            current_pose = pose2;
            break;
          case 2:
            current_pose = pose3;
            break;
          default: // case 0
            current_pose = pose1;
            break;
        }
        RCLCPP_INFO(LOGGER, "Max ticks reached.");
        RCLCPP_INFO(LOGGER, "Publishing pose %d", pose_num);
        RCLCPP_INFO(LOGGER, "Position {%.3f, %.3f, %.3f}",
          current_pose.position.x,
          current_pose.position.y,
          current_pose.position.z
        );
        RCLCPP_INFO(LOGGER, "Quaternion {%.3f, %.3f, %.3f, %.3f}",
          current_pose.orientation.w,
          current_pose.orientation.x,
          current_pose.orientation.y,
          current_pose.orientation.z
        );
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
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}