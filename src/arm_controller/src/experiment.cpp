// C++
#include <vector>
#include <chrono>
#include <memory>
#include <cmath>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/std_msgs/msg/float64.hpp>
#include "arm_controller/msg/path_and_execution_timing.hpp"
#include "arm_controller/msg/constrained_pose.hpp"

using namespace arm_controller::msg;
using namespace geometry_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;

/**
 * Experiment behavior interface.
 */
enum ExperimentBehavior
{
  CONSTRAINT_EXP,
  HIGH_FREQ_EXP,
  ELLIPSE_TRACE_EXP,
  FORCE_SENSOR_EXP
};

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

    void set_max_ticks(int max_ticks)
    {
      this->max_ticks = max_ticks;
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
      ticks++;
      if(ticks == max_ticks)
      {
        ticks = 0;
        pose_num = 1 + (rand() % 3);
      }
    }

  private:
    int pose_num = 1;
    int ticks = 0;
    int max_ticks = 500;
    // poses for testing the arm's motion
    geometry_msgs::msg::Pose pose1;
    geometry_msgs::msg::Pose pose2;
    geometry_msgs::msg::Pose pose3;
};


class CircleTraceExperiment
{
  public:
    CircleTraceExperiment() {
      waist_angle = compute_waist_position();
    }

    void set_max_ticks(int max_ticks) {
      this->max_ticks = max_ticks;
    }

    void publish(
      rclcpp::Publisher<Pose>::SharedPtr pub,
      const rclcpp::Logger LOGGER
    )
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
      pub->publish(msg);

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

  private:
    // The position on the ellipse
    double theta = 0;
    double theta_step = M_PI / 256.0;
    // angle the waist needs to be in (basically yaw angle)
    double waist_angle;
    int ticks = 0;
    int max_ticks = 500;


    double compute_waist_position() 
    {
      double num = (1.0/8.0) * std::cos(theta);
      double dem = std::sqrt((1.0/64.0) * std::cos(theta) + (0.223*0.223));
      return std::acos((double) num / dem);
    }
};

/**
 * This class will publish random values to represent the
 * force being applied to the arm.
 */
class ForceSensorExperiment
{
  public:
    ForceSensorExperiment(){}

    void set_max_ticks(int max_ticks) {
      this->max_ticks = max_ticks;
    }

    void publish(
      rclcpp::Publisher<Float64>::SharedPtr pub,
      const rclcpp::Logger LOGGER
    )
    {
      auto msg = Float64();
      if(ticks == max_ticks)
      {
        ticks = 0;
        if(current_force >= 2) {
          current_force = 0;
        }
        else {
          double increment = (double) rand() / RAND_MAX;
          current_force += increment;
          RCLCPP_INFO(LOGGER, "Increasing force by %f", increment);
        }
      }
      else ticks++;
      msg.data = current_force;
      RCLCPP_INFO(LOGGER, "Publishing value F = %f", current_force);
      pub->publish(msg);
    }

  private:
    double current_force = 0;
    int ticks = 0;
    int max_ticks = 500;
};

class Experiment : public rclcpp::Node
{
  public:
    Experiment() : Node("experiment")
    {
      int max_ticks = 500;
      std::string exp_type = "";

      constraint_publisher_ = this->create_publisher<ConstrainedPose>(
        "joy_target_pose",
        1
      );

      pose_publisher_ = this->create_publisher<Pose>(
        "high_freq_publisher",
        10
      );

      force_publisher_ = this->create_publisher<Float64>(
        "force",
        10
      );

      this->declare_parameter("delay",     5.0);
      this->declare_parameter("duration",  300.0);
      this->declare_parameter("frequency", 5.0);
      this->declare_parameter("max_ticks", 500);
      this->declare_parameter("exp_type", "");

      this->get_parameter("delay",     delay_in_seconds);
      this->get_parameter("duration",  duration_in_seconds);
      this->get_parameter("frequency", frequency_in_seconds);
      this->get_parameter("max_ticks", max_ticks);
      this->get_parameter("exp_type", exp_type);

      this->factoryMethod(exp_type, max_ticks);

      delay = std::chrono::duration<double>(delay_in_seconds);
      duration = std::chrono::duration<double>(duration_in_seconds);
      frequency = std::chrono::duration<double>(frequency_in_seconds);

      delay_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::duration<double>>(delay),
        std::bind(&Experiment::start_timer, this)
      );
    }    

    void start_timer()
    {
      delay_->cancel();

      timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::duration<double>>(frequency),
        std::bind(&Experiment::run_experiment, this)
      );

      stop_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::duration<double>>(duration), 
        std::bind(&Experiment::stop_experiment, this)
      );
    }

    void run_experiment()
    {
      switch (expBehavior)
      {
      case CONSTRAINT_EXP:
        constraint_exp.publish(constraint_publisher_);
        break;
      
      case HIGH_FREQ_EXP:
        high_freq_exp.publish(pose_publisher_, LOGGER);
        break;
      
      case ELLIPSE_TRACE_EXP:
        circ_trace_exp.publish(pose_publisher_, LOGGER);
        break;

      case FORCE_SENSOR_EXP:
        force_sensor_exp.publish(force_publisher_, LOGGER);
        break;

      default:
        break;
      }
    }

    void stop_experiment()
    {
      RCLCPP_INFO(this->get_logger(), "Canceling timer");
      timer_->cancel();
    }

  private:
    rclcpp::Publisher<ConstrainedPose>::SharedPtr constraint_publisher_;
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher_;
    rclcpp::Publisher<Float64>::SharedPtr force_publisher_;

    rclcpp::TimerBase::SharedPtr delay_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;

    double delay_in_seconds;
    double duration_in_seconds;
    double frequency_in_seconds;

    std::chrono::duration<double> delay;
    std::chrono::duration<double> duration;
    std::chrono::duration<double> frequency;

    ExperimentBehavior expBehavior;

    ConstraintExperiment constraint_exp;
    HighFreqExperiment high_freq_exp;
    CircleTraceExperiment circ_trace_exp;
    ForceSensorExperiment force_sensor_exp;

    const rclcpp::Logger LOGGER = rclcpp::get_logger("experiment");

    /* Factory methods */
    /**
     * Returns the desired experiment type.
     */
    void factoryMethod(
      std::string exp_type, int max_ticks
    )
    {
      if (exp_type.compare("constraint-exp") == 0)
      {
        constraint_exp = ConstraintExperiment();
        expBehavior = CONSTRAINT_EXP;
      }
      else if (exp_type.compare("ellipse-trace") == 0)
      {
        circ_trace_exp = CircleTraceExperiment();
        circ_trace_exp.set_max_ticks(max_ticks);
        expBehavior = ELLIPSE_TRACE_EXP;
      }
      else if (exp_type.compare("force-exp") == 0)
      {
        force_sensor_exp = ForceSensorExperiment();
        force_sensor_exp.set_max_ticks(max_ticks);
        expBehavior = FORCE_SENSOR_EXP;
      }
      else
      {
        high_freq_exp = HighFreqExperiment();
        high_freq_exp.set_max_ticks(max_ticks);
        expBehavior = HIGH_FREQ_EXP;
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Experiment>());
  rclcpp::shutdown();
  return 0;
}