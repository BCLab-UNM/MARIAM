// C++
#include <vector>
#include <chrono>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "arm_controller/msg/path_and_execution_timing.hpp"
#include "arm_controller/msg/constrained_pose.hpp"

using namespace arm_controller::msg;
using namespace std::chrono;
using std::placeholders::_1;

class ConstraintExperiment : public rclcpp::Node
{
  public:
    ConstraintExperiment() : Node("constraint_experiment")
    {
      pose1.position.x = 0.250048;
      pose1.position.y = 0;
      pose1.position.z = 0.098;
      pose1.orientation.x =  0.000;
      pose1.orientation.y =  0.000;
      pose1.orientation.z =  0.000;
      pose1.orientation.w =  1.000;

      pose2.position.x = 0.250048;
      pose2.position.y = 0;
      pose2.position.z = 0.244;
      pose2.orientation.x =  0.000;
      pose2.orientation.y =  0.000;
      pose2.orientation.z =  0.000;
      pose2.orientation.w =  1.000;

      pose_publisher_ = this->create_publisher<ConstrainedPose>(
        "joy_target_pose",
        10
      );

      timer_ = this->create_wall_timer(
        15s, std::bind(&ConstraintExperiment::run_experiment, this));
      
      stop_timer_ = this->create_wall_timer(
        publish_duration_, std::bind(
          &ConstraintExperiment::stop_experiment,
          this
        )
      );
    }

    void run_experiment()
    {
      auto msg = ConstrainedPose();
      if(use_first_pose) msg.pose = pose1;
      else msg.pose = pose2;
      msg.use_plane_constraint = true;
      RCLCPP_INFO(this->get_logger(), "Publishing a new pose");
      pose_publisher_->publish(msg);
      use_first_pose = !use_first_pose;
    }

    void stop_experiment()
    {
      RCLCPP_INFO(this->get_logger(), "Canceling timer");
      timer_->cancel();
    }

  private:
    rclcpp::Publisher<ConstrainedPose>::SharedPtr pose_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    seconds publish_duration_ = 300s;
    bool use_first_pose = true;
    
    // poses for testing the arm's motion
    geometry_msgs::msg::Pose pose1;
    geometry_msgs::msg::Pose pose2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConstraintExperiment>());
  rclcpp::shutdown();
  return 0;
}