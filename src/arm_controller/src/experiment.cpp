// C++
#include <vector>
#include <chrono>
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "arm_controller/msg/path_and_execution_timing.hpp"
#include "arm_controller/msg/constrained_pose.hpp"

using namespace arm_controller::msg;
using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * This class contains poses for testing the trajectory
 * of the arm when moving up and down.
 */
class ConstraintExperiment
{
  public:
    ConstraintExperiment()
    {
      pose1.position.x = 0.0;
      pose1.position.y = 0.250048;
      pose1.position.z = 0.098;
      pose1.orientation.x =  0.000;
      pose1.orientation.y =  0.000;
      pose1.orientation.z =  0.707;
      pose1.orientation.w =  0.707;

      pose2.position.x = 0.0;
      pose2.position.y = 0.250048;
      pose2.position.z = 0.244;
      pose2.orientation.x =  0.000;
      pose2.orientation.y =  0.000;
      pose2.orientation.z =  0.707;
      pose2.orientation.w =  0.707;
    }

    void publish(rclcpp::Publisher<ConstrainedPose>::SharedPtr pose_publisher)
    {
      auto msg = ConstrainedPose();
      if(use_first_pose) msg.pose = pose1;
      else msg.pose = pose2;
      msg.use_plane_constraint = true;
      msg.pose_name = "none";
      pose_publisher->publish(msg);
      use_first_pose = !use_first_pose;
    }

  private:
    bool use_first_pose = true;
    // poses for testing the arm's motion
    geometry_msgs::msg::Pose pose1;
    geometry_msgs::msg::Pose pose2;
};

/**
 * This class contains poses that test the arm's trajectory
 * when a high number of goal poses are published.
 */
class HighFreqExperiment
{
  public:
    HighFreqExperiment()
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

      srand(42);
    }

    void publish(
      rclcpp::Publisher<Pose>::SharedPtr pub,
      const rclcpp::Logger LOGGER
    )
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

      pub->publish(msg);
      if(ticks == MAX_TICKS)
      {
        ticks = 0;
        pose_num = 1 + (rand() % 3);
      }
      else ticks++;
    }

  private:
    int pose_num = 1;
    int ticks = 0;
    const int MAX_TICKS = 100000;
    // poses for testing the arm's motion
    geometry_msgs::msg::Pose pose1;
    geometry_msgs::msg::Pose pose2;
    geometry_msgs::msg::Pose pose3;
};

class Experiment : public rclcpp::Node
{
  public:
    Experiment() : Node("experiment")
    {
      pose_publisher_ = this->create_publisher<ConstrainedPose>(
        "joy_target_pose",
        1
      );

      pose_publisher2_ = this->create_publisher<Pose>(
        "high_freq_publisher",
        10
      );

      /* Parameter */
      this->declare_parameter("delay", 5.0);
      this->declare_parameter("duration", 300.0);
      this->declare_parameter("frequency", 5.0);

      this->get_parameter("delay", delay_in_seconds);
      this->get_parameter("duration", duration_in_seconds);
      this->get_parameter("frequency", frequency_in_seconds);

      delay = std::chrono::duration<double>(delay_in_seconds);
      duration = std::chrono::duration<double>(duration_in_seconds);
      frequency = std::chrono::duration<double>(frequency_in_seconds);

      delay_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::seconds>(delay),
        std::bind(&Experiment::start_timer, this)
      );
    }    

    void start_timer()
    {
      delay_->cancel();

      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::seconds>(frequency), 
        std::bind(&Experiment::run_experiment, this)
      );

      stop_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::seconds>(duration), 
        std::bind(&Experiment::stop_experiment, this)
      );
    }

    void run_experiment()
    {
      // RCLCPP_INFO(this->get_logger(), "Publishing a new pose");
      high_freq_exp.publish(this->pose_publisher2_, LOGGER);
    }

    void stop_experiment()
    {
      RCLCPP_INFO(this->get_logger(), "Canceling timer");
      timer_->cancel();
    }

  private:
    rclcpp::Publisher<ConstrainedPose>::SharedPtr pose_publisher_;
    ConstraintExperiment constraint_experiment;
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher2_;
    HighFreqExperiment high_freq_exp;

    rclcpp::TimerBase::SharedPtr delay_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;

    double delay_in_seconds;
    double duration_in_seconds;
    double frequency_in_seconds;

    std::chrono::duration<double> delay;
    std::chrono::duration<double> duration;
    std::chrono::duration<double> frequency;

    const rclcpp::Logger LOGGER = rclcpp::get_logger("experiment"); 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Experiment>());
  rclcpp::shutdown();
  return 0;
}