#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <chrono>

class RobotFollower : public rclcpp::Node
{
public:
    RobotFollower() : Node("robot_follower")
    {
        // Parameters
        desired_distance_ = 0.99;  // meters
        distance_tolerance_ = 0.03;  // +/- 3cm tolerance
        heading_tolerance_ = 0.05;   // +/- ~3 degrees (0.05 rad = ~2.9 degrees)
        max_linear_vel_ = 0.5;     // m/s
        max_angular_vel_ = 1.0;    // rad/s
        
        // PID Parameters (tuned for low-speed following)
        kp_linear_ = 0.8;      // Proportional gain for linear velocity
        ki_linear_ = 0.1;      // Integral gain for linear velocity  
        kd_linear_ = 0.05;     // Derivative gain for linear velocity
        
        kp_angular_ = 2.0;     // Proportional gain for angular velocity
        
        // PID state variables
        prev_error_ = 0.0;
        integral_error_ = 0.0;
        prev_time_ = this->get_clock()->now();
        
        // Robot poses
        leader_pose_received_ = false;
        follower_pose_received_ = false;
        
        // Publishers and Subscribers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/ross/cmd_vel", 10);
        
        leader_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_monica_pose", 10,
            std::bind(&RobotFollower::leader_pose_callback, this, std::placeholders::_1));
        
        follower_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_ross_pose", 10,
            std::bind(&RobotFollower::follower_pose_callback, this, std::placeholders::_1));
        
        // Control timer (50 Hz control loop)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RobotFollower::control_callback, this));
        
        // Logging counter
        log_counter_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Robot Follower Node Started");
        RCLCPP_INFO(this->get_logger(), "Target distance: %.3fm", desired_distance_);
        RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.3fm/s", max_linear_vel_);
    }

private:
    void leader_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        leader_pose_ = *msg;
        leader_pose_received_ = true;
    }

    void follower_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        follower_pose_ = *msg;
        follower_pose_received_ = true;
    }

    void control_callback()
    {
        // Check if we have both poses
        if (!leader_pose_received_ || !follower_pose_received_)
        {
            return;
        }
        
        // Extract positions
        double leader_x = leader_pose_.position.x;
        double leader_y = leader_pose_.position.y;
        
        double follower_x = follower_pose_.position.x;
        double follower_y = follower_pose_.position.y;
        
        // Calculate current distance
        double dx = leader_x - follower_x;
        double dy = leader_y - follower_y;
        double current_distance = sqrt(dx*dx + dy*dy);
        
        // Calculate distance error with tolerance deadband
        double distance_error = current_distance - desired_distance_;
        
        // Apply distance tolerance (deadband)
        double distance_control_error = 0.0;
        if (abs(distance_error) > distance_tolerance_)
        {
            // Remove tolerance from error to maintain smooth control
            distance_control_error = (distance_error > 0) ? 
                (distance_error - distance_tolerance_) : 
                (distance_error + distance_tolerance_);
        }
        
        // Calculate time step
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - prev_time_).seconds();
        prev_time_ = current_time;
        
        if (dt > 0.1)  // Reset if too much time has passed (initial startup)
        {
            dt = 0.02;
            prev_error_ = distance_control_error;
            integral_error_ = 0.0;
        }
        
        // PID control for linear velocity (using control error, not raw error)
        integral_error_ += distance_control_error * dt;
        double derivative_error = (dt > 0) ? (distance_control_error - prev_error_) / dt : 0.0;
        
        // Integral windup protection
        double max_integral = (ki_linear_ > 0) ? 0.3 / ki_linear_ : 0;
        integral_error_ = std::max(-max_integral, std::min(max_integral, integral_error_));
        
        // Calculate linear velocity command
        double linear_vel = kp_linear_ * distance_control_error + 
                           ki_linear_ * integral_error_ + 
                           kd_linear_ * derivative_error;
        
        // Clamp linear velocity
        linear_vel = std::max(-max_linear_vel_, std::min(max_linear_vel_, linear_vel));
        
        // Calculate desired heading (toward leader)
        double desired_heading = atan2(dy, dx);
        
        // Get current heading from follower pose
        double follower_heading = quaternion_to_yaw(follower_pose_.orientation);
        
        // Calculate heading error (normalized to [-pi, pi])
        double heading_error = desired_heading - follower_heading;
        heading_error = atan2(sin(heading_error), cos(heading_error));
        
        // Apply heading tolerance (deadband)
        double heading_control_error = 0.0;
        if (abs(heading_error) > heading_tolerance_)
        {
            // Remove tolerance from error to maintain smooth control
            heading_control_error = (heading_error > 0) ? 
                (heading_error - heading_tolerance_) : 
                (heading_error + heading_tolerance_);
        }
        
        // Calculate angular velocity (simple proportional control)
        double angular_vel = kp_angular_ * heading_control_error;
        angular_vel = std::max(-max_angular_vel_, std::min(max_angular_vel_, angular_vel));
        
        // Create and publish command
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear_vel;
        cmd.angular.z = angular_vel;
        
        cmd_pub_->publish(cmd);
        
        // Store previous error (using control error for PID continuity)
        prev_error_ = distance_control_error;
        
        // Log status (every 50 iterations = 1 second)
        log_counter_++;
        if (log_counter_ % 50 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "Distance: %.3fm (error: %.3fm, in_tol: %s), Heading error: %.3frad (in_tol: %s), Linear vel: %.3fm/s, Angular vel: %.3frad/s",
                current_distance, distance_error, 
                (abs(distance_error) <= distance_tolerance_) ? "Y" : "N",
                heading_error,
                (abs(heading_error) <= heading_tolerance_) ? "Y" : "N",
                linear_vel, angular_vel);
        }
    }

    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& quat)
    {
        // Convert quaternion to yaw angle
        double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
        double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
        return atan2(siny_cosp, cosy_cosp);
    }

    // Parameters
    double desired_distance_;
    double distance_tolerance_;
    double heading_tolerance_;
    double max_linear_vel_;
    double max_angular_vel_;
    
    // PID parameters
    double kp_linear_, ki_linear_, kd_linear_;
    double kp_angular_;
    
    // PID state variables
    double prev_error_;
    double integral_error_;
    rclcpp::Time prev_time_;
    
    // Robot poses
    geometry_msgs::msg::Pose leader_pose_;
    geometry_msgs::msg::Pose follower_pose_;
    bool leader_pose_received_;
    bool follower_pose_received_;
    
    // ROS2 components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr leader_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr follower_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Logging counter
    int log_counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotFollower>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    // Stop the robot before shutting down
    geometry_msgs::msg::Twist stop_cmd;
    // stop_cmd is already zero-initialized
    
    rclcpp::shutdown();
    return 0;
}