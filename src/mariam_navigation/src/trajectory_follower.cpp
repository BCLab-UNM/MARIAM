#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <nav_msgs/msg/path.hpp>
#include "pid_controller.hpp"
#include "cubic_spline.hpp"
#include "cubic_polynomial.hpp"


class TrajectoryFollower : public rclcpp::Node {
  public:
    TrajectoryFollower() : Node("trajectory_follower"),
      // PID parameters
      pid_x_(2.0, 1.0, 0.5),
      pid_y_(2.0, 1.0, 0.5),
      pid_theta_(3.0, 0.0, 0.8) {
        
      // Declare parameters
      this->declare_parameter("x_0", 0.0);
      this->declare_parameter("y_0", 0.0);
      this->declare_parameter("x_f", 3.0);
      this->declare_parameter("y_f", 0.0);
      this->declare_parameter("trajectory_duration", 10.0);
      this->declare_parameter("control_frequency", 50.0);
      this->declare_parameter("max_linear_vel", 0.5);
      this->declare_parameter("max_angular_vel", 1.0);
      // these are for splines only
      this->declare_parameter("num_waypoints", 5);
      this->declare_parameter("parabola_coeff", -1.5);
        
      this->robot_name = this->get_namespace(); // make namespace an std::string
      std::string pose_topic = robot_name == "/ross" ? "/world_ross_pose" : "/world_monica_pose";

      RCLCPP_INFO(this->get_logger(),
        "Robot name: %s",
        robot_name.c_str()
      );

      // Publishers and subscribers
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        pose_topic,
        10,
        std::bind(&TrajectoryFollower::poseCallback, this, std::placeholders::_1)
      );
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory_path", 10);
      
      RCLCPP_INFO(this->get_logger(),
        "Subscribing to vicon pose: %s",
        pose_topic.c_str()
      );

      // Generate trajectory
      // generatePolynomialTrajectory();
      generateSplineTrajectory();
      publishTrajectoryPath();
      
      // Start control timer
      double control_freq = this->get_parameter("control_frequency").as_double();
      auto timer_period = std::chrono::duration<double>(1.0 / control_freq);
      control_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&TrajectoryFollower::controlLoop, this)
      );
      
      trajectory_start_time_ = this->now();
      
      RCLCPP_INFO(this->get_logger(),
        "TrajectoryFollower initialized. Starting trajectory following.");
    }

  private:
    /**
     * This function will generate a trajectory between two points,
     * which is represented as a cubic polynomial.
     * 
     * @param start_pos: the initial position.
     * @param start_vel: the initial velocity.
     * @param end_pos: the final position.
     * @param end_vel: the final velocity.
     * @param duration: the amount of time to traverse the trajectory.
     */
    CubicPolynomial generate_cubic_trajectory(
      double start_pos, double start_vel,
      double end_pos, double end_vel,
      double duration) {
      
      CubicPolynomial p;
      // T = short hand for total duration
      double T_squared = duration * duration;
      double T_cubed = T_squared * duration;
      
      /** the cubic polynomial is of the form
      *     ax^3 + bx^2 + cx + d
      */ 
      p.d = start_pos; // p(0) = start_pos = d
      p.c = start_vel; // p'(0) = start_vel = c
      // These formulas can be derived by solving a system
      // of equations (confirmed through hand calculations)
      p.b = (3.0 * (end_pos - start_pos) - duration * (2.0 * start_vel + end_vel)) / T_squared; 
      p.a = (2.0 * (start_pos - end_pos) + duration * (start_vel + end_vel)) / T_cubed;
      
      return p;
    }

    void generateSplineTrajectory() {
      double x_0 = this->get_parameter("x_0").as_double();
      double y_0 = this->get_parameter("y_0").as_double();
      double x_f = this->get_parameter("x_f").as_double();
      double y_f = this->get_parameter("y_f").as_double();
      double duration = this->get_parameter("trajectory_duration").as_double();
      int num_waypoints = this->get_parameter("num_waypoints").as_int();
      double a = this->get_parameter("parabola_coeff").as_double();
      
      // Generate waypoints with a sinusoidal curve
      std::vector<double> t_waypoints, x_waypoints, y_waypoints;
      
      // add additional waypoints
      for (int i = 0; i < num_waypoints; ++i) {
        // progress is the path parameter and is from [0, 1]
        double progress = static_cast<double>(i) / (num_waypoints);
        double t = progress * duration;
        double x = x_0 + progress * (x_f - x_0);
        double y = y_0 + progress * (y_f - y_0) + a * progress * progress;
        
        t_waypoints.push_back(t);
        x_waypoints.push_back(x);
        y_waypoints.push_back(y);
      }

      t_waypoints.push_back(duration);
      x_waypoints.push_back(x_f);
      y_waypoints.push_back(y_f);
      
      // Create splines
      trajectory_.x_spline.setPoints(t_waypoints, x_waypoints);
      trajectory_.y_spline.setPoints(t_waypoints, y_waypoints);
      trajectory_.duration = duration;
      
      RCLCPP_INFO(this->get_logger(), "Generated spline trajectory with %d waypoints over %.2f seconds",
                  num_waypoints, duration);
    }
    
    void generatePolynomialTrajectory() {
      double x_0 = this->get_parameter("x_0").as_double();
      double y_0 = this->get_parameter("y_0").as_double();
      double x_f = this->get_parameter("x_f").as_double();
      double y_f = this->get_parameter("y_f").as_double();
      double duration = this->get_parameter("trajectory_duration").as_double();
      
      // Generate cubic polynomial for x and y coordinates
      // Assuming zero initial and final velocities for smooth start/stop
      // trajectory_.x_poly = generate_cubic_trajectory(x_0, 0.0, x_f, 0.0, duration);
      // trajectory_.y_poly = generate_cubic_trajectory(y_0, 0.0, y_f, 0.0, duration);
      trajectory_.duration = duration;
      
      RCLCPP_INFO(this->get_logger(), "Generated trajectory from (%.2f, %.2f) to (%.2f, %.2f) over %.2f seconds",
                  x_0, y_0, x_f, y_f, duration);
    }
    
    void publishTrajectoryPath() {
      nav_msgs::msg::Path path;
      path.header.frame_id = "world";  // Adjust frame as needed
      path.header.stamp = this->now();
      
      // Sample the trajectory at regular intervals for visualization
      int num_samples = 100;
      for (int i = 0; i <= num_samples; ++i) {
        double t = (static_cast<double>(i) / num_samples) * trajectory_.duration;
        auto point = trajectory_.getPoint(t);
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = this->now();
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        
        // Calculate orientation from velocity direction
        double theta = std::atan2(point.vy, point.vx);
        pose.pose.orientation.z = std::sin(theta / 2.0);
        pose.pose.orientation.w = std::cos(theta / 2.0);
        
        path.poses.push_back(pose);
      }
      
      path_pub_->publish(path);
    }
    
    void poseCallback(const geometry_msgs::msg::Pose &msg) {
      current_pose_ = msg;
      pose_received_ = true;
    }
    
    void controlLoop() {
      if (!pose_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "No pose received from Vicon system");
        return;
      }
      
      // Calculate time elapsed since trajectory start
      double elapsed_time = (this->now() - trajectory_start_time_).seconds();
      
      // Check if trajectory is complete
      if (elapsed_time > trajectory_.duration) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel_pub_->publish(cmd_vel);  // Stop the robot
        RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory completed!");
        return;
      }
      
      // Get desired position and velocity from trajectory
      auto desired = trajectory_.getPoint(elapsed_time);
      
      // Current robot position
      double current_x = current_pose_.position.x;
      double current_y = current_pose_.position.y;
      
      // Extract current orientation (assuming quaternion represents yaw only)
      double current_theta = 2.0 * std::atan2(current_pose_.orientation.z,
                                              current_pose_.orientation.w);
      
      if (this->robot_name == "/monica") {
        // make it think the heading is facing behind monica
        current_theta += M_PI;
        // Normalize current heading to [-pi, pi]
        while (current_theta > M_PI) current_theta -= 2 * M_PI;
        while (current_theta < -M_PI) current_theta += 2 * M_PI;
      }

      // Calculate position errors
      double error_x = desired.x - current_x;
      double error_y = desired.y - current_y;
      
      // Calculate desired heading from desired velocity
      double desired_theta = std::atan2(desired.vy, desired.vx);

      double error_theta = desired_theta - current_theta;
      
      // Normalize angle error to [-pi, pi]
      while (error_theta > M_PI) error_theta -= 2 * M_PI;
      while (error_theta < -M_PI) error_theta += 2 * M_PI;
      
      // PID control (simplified approach)
      double dt = 1.0 / this->get_parameter("control_frequency").as_double();
      
      // Transform position errors to robot frame for better control
      double error_forward = error_x * std::cos(current_theta) + error_y * std::sin(current_theta);
      double error_lateral = -error_x * std::sin(current_theta) + error_y * std::cos(current_theta);
      
      // Compute control commands
      double linear_vel = pid_x_.compute(error_forward, dt);
      double angular_vel = pid_theta_.compute(error_theta, dt) + 
                          0.5 * pid_y_.compute(error_lateral, dt);  // Lateral error contributes to turning
      
      // Apply velocity limits
      double max_linear = this->get_parameter("max_linear_vel").as_double();
      double max_angular = this->get_parameter("max_angular_vel").as_double();
      
      linear_vel = std::clamp(linear_vel, -max_linear, max_linear);
      angular_vel = std::clamp(angular_vel, -max_angular, max_angular);

      // negate the linear velocity if it's monica
      if (this->robot_name == "/monica") {
        linear_vel = linear_vel * -1.0;
      }
      
      // Publish command
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = linear_vel;
      cmd_vel.angular.z = angular_vel;
      cmd_vel_pub_->publish(cmd_vel);
      
      // Debug output
      RCLCPP_DEBUG(this->get_logger(), 
                  "t=%.2f, desired=(%.2f,%.2f), current=(%.2f,%.2f), error=(%.2f,%.2f), cmd=(%.2f,%.2f)",
                  elapsed_time, desired.x, desired.y, current_x, current_y,
                  error_x, error_y, linear_vel, angular_vel);
    }
    
    // Member variables
    // Trajectory2D trajectory_;
    SplineTrajectory2D trajectory_;
    PIDController pid_x_, pid_y_, pid_theta_;
    std::string robot_name;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    geometry_msgs::msg::Pose current_pose_;
    bool pose_received_ = false;
    rclcpp::Time trajectory_start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}