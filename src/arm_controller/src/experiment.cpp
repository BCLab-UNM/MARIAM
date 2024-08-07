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
using namespace std::chrono;
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
    }

    void publish(rclcpp::Publisher<Pose>::SharedPtr pub)
    {
      auto msg = Pose();

      if(use_first_pose)
      {
        msg.position.x = pose1.position.x;
        msg.position.y = pose1.position.y;
        msg.position.z = pose1.position.z;
        msg.orientation.w = pose1.orientation.w;
        msg.orientation.x = pose1.orientation.x;
        msg.orientation.y = pose1.orientation.y;
        msg.orientation.z = pose1.orientation.z;
      }
      else 
      {
        msg.position = pose2.position;
        msg.orientation = pose2.orientation;
      }

      pub->publish(msg);
      if(ticks == MAX_TICKS)
      {
        ticks = 0;
        use_first_pose = !use_first_pose;
      }
      else ticks++;
    }

  private:
    bool use_first_pose = true;
    int ticks = 0;
    const int MAX_TICKS = 1000;
    // poses for testing the arm's motion
    geometry_msgs::msg::Pose pose1;
    geometry_msgs::msg::Pose pose2;
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
        1
      );

      timer_ = this->create_wall_timer(
        this->FREQUENCY, 
        std::bind(&Experiment::run_experiment, this)
      );
      
      stop_timer_ = this->create_wall_timer(
        this->PUBLISH_DURATION,
        std::bind(&Experiment::stop_experiment, this)
      );
    }

    void run_experiment()
    {
      RCLCPP_INFO(this->get_logger(), "Publishing a new pose");
      high_freq_exp.publish(this->pose_publisher2_);
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

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    const seconds PUBLISH_DURATION = 300s;
    const nanoseconds FREQUENCY = 2000000ns;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Experiment>());
  rclcpp::shutdown();
  return 0;
}