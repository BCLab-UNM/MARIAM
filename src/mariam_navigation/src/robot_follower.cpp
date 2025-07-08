#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>

class RobotFollower : public rclcpp::Node
{
public:
    RobotFollower() : Node("robot_follower")
    {
        // Parameters
        following_distance_ = 0.5;  // meters
        position_tolerance_ = 0.05;  // meters
        angle_tolerance_ = 0.1;  // radians
        
        // Velocity limits (matching Monica's constraints)
        max_linear_vel_ = 0.2;
        min_linear_vel_ = -0.2;
        max_angular_vel_ = 0.4;
        min_angular_vel_ = -0.4;
        
        // Control gains
        linear_gain_ = 1.0;
        angular_gain_ = 2.0;
        
        // Initialize pose pointers
        monica_pose_ = nullptr;
        ross_pose_ = nullptr;
        
        // Initialize timestamps
        last_monica_time_ = this->now();
        last_ross_time_ = this->now();
        poses_received_ = false;
        
        // Subscribers
        monica_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/monica/pose", 10,
            std::bind(&RobotFollower::monicaPoseCallback, this, std::placeholders::_1));
        
        ross_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ross/pose", 10,
            std::bind(&RobotFollower::rossPoseCallback, this, std::placeholders::_1));
        
        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/ross/cmd_vel", 10);
        
        // Timers
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&RobotFollower::controlLoop, this));
        
        safety_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 2 Hz
            std::bind(&RobotFollower::safetyCheck, this));
        
        RCLCPP_INFO(this->get_logger(), "Robot Follower Node initialized");
    }

    // Public method to stop the robot
    void stopRobot()
    {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
    }

private:
    // Parameters
    double following_distance_;
    double position_tolerance_;
    double angle_tolerance_;
    double max_linear_vel_;
    double min_linear_vel_;
    double max_angular_vel_;
    double min_angular_vel_;
    double linear_gain_;
    double angular_gain_;
    
    // Pose storage
    std::shared_ptr<geometry_msgs::msg::Pose> monica_pose_;
    std::shared_ptr<geometry_msgs::msg::Pose> ross_pose_;
    rclcpp::Time last_monica_time_;
    rclcpp::Time last_ross_time_;
    bool poses_received_;
    
    // ROS components
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr monica_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ross_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    
    void monicaPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        monica_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
        last_monica_time_ = this->now();
        poses_received_ = true;
    }
    
    void rossPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        ross_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
        last_ross_time_ = this->now();
        poses_received_ = true;
    }
    
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& quaternion)
    {
        tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        return tf2::getYaw(q);
    }
    
    std::pair<double, double> calculateTargetPosition(const geometry_msgs::msg::Pose& monica_pose)
    {
        double monica_yaw = quaternionToYaw(monica_pose.orientation);
        
        // Calculate target position 0.5m behind Monica
        double target_x = monica_pose.position.x - following_distance_ * std::cos(monica_yaw);
        double target_y = monica_pose.position.y - following_distance_ * std::sin(monica_yaw);
        
        return std::make_pair(target_x, target_y);
    }
    
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    double clamp(double value, double min_val, double max_val)
    {
        return std::max(min_val, std::min(max_val, value));
    }
    
    void controlLoop()
    {
        if (!monica_pose_ || !ross_pose_) {
            return;
        }
        
        // Calculate target position
        auto [target_x, target_y] = calculateTargetPosition(*monica_pose_);
        
        // Current Ross position
        double ross_x = ross_pose_->position.x;
        double ross_y = ross_pose_->position.y;
        double ross_yaw = quaternionToYaw(ross_pose_->orientation);
        
        // Calculate error
        double error_x = target_x - ross_x;
        double error_y = target_y - ross_y;
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
        
        // Calculate desired heading (angle to target)
        double target_angle = std::atan2(error_y, error_x);
        double angle_error = normalizeAngle(target_angle - ross_yaw);
        
        // Create command velocity
        geometry_msgs::msg::Twist cmd_vel;
        
        // Only move if error is significant
        if (distance_error > position_tolerance_) {
            // Linear velocity proportional to distance error
            cmd_vel.linear.x = linear_gain_ * distance_error;
            
            // Angular velocity proportional to angle error
            cmd_vel.angular.z = angular_gain_ * angle_error;
            
            // Apply velocity limits
            cmd_vel.linear.x = clamp(cmd_vel.linear.x, min_linear_vel_, max_linear_vel_);
            cmd_vel.angular.z = clamp(cmd_vel.angular.z, min_angular_vel_, max_angular_vel_);
        } else {
            // Stop if close enough
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        
        // Publish command
        cmd_vel_pub_->publish(cmd_vel);
        
        // Debug info
        RCLCPP_DEBUG(this->get_logger(),
            "Distance error: %.3f m, Angle error: %.3f rad, "
            "Cmd: linear=%.3f, angular=%.3f",
            distance_error, angle_error, cmd_vel.linear.x, cmd_vel.angular.z);
    }
    
    void safetyCheck()
    {
        rclcpp::Time current_time = this->now();
        rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(2.0);
        
        if (!poses_received_ ||
            (current_time - last_monica_time_) > timeout_duration ||
            (current_time - last_ross_time_) > timeout_duration) {
            
            // Stop the robot
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_cmd);
            
            if (!poses_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Waiting for pose messages...");
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Pose messages are stale, stopping robot for safety");
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotFollower>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    // Stop the robot on shutdown
    node->stopRobot();
    RCLCPP_INFO(node->get_logger(), "Shutting down robot follower");
    
    rclcpp::shutdown();
    return 0;
}