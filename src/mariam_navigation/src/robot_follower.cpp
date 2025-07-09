#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class RobotFollower : public rclcpp::Node
{
public:
    RobotFollower() : Node("robot_follower")
    {
        // Parameters
        following_distance_ = 0.5;  // meters
        position_tolerance_ = 0.1;  // meters - increased tolerance
        angle_tolerance_ = 0.1;  // radians
        
        // Velocity limits (matching Monica's constraints)
        max_linear_vel_ = 0.2;
        min_linear_vel_ = -0.2;
        max_angular_vel_ = 0.4;
        min_angular_vel_ = -0.4;
        
        // Control gains
        linear_gain_ = 1.0;
        angular_gain_ = 2.0;
        
        // TF2 setup - separate buffers for each robot's namespaced topics
        monica_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        ross_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        
        // Subscribe to namespaced TF topics
        monica_tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/monica/tf", 10,
            std::bind(&RobotFollower::monica_tf_callback, this, std::placeholders::_1));
            
        ross_tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/ross/tf", 10,
            std::bind(&RobotFollower::ross_tf_callback, this, std::placeholders::_1));
        
        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/ross/cmd_vel", 10);
        
        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&RobotFollower::controlLoop, this));
        
        // Safety timer
        safety_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 2 Hz
            std::bind(&RobotFollower::safetyCheck, this));
        
        // Initialize safety tracking
        transforms_initialized_ = false;
        last_monica_transform_time_ = this->now();
        last_ross_transform_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Robot Follower Node initialized with custom TF topics");
    }

    void monica_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            monica_tf_buffer_->setTransform(transform, "monica_tf", false);
        }
    }
    
    void ross_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            ross_tf_buffer_->setTransform(transform, "ross_tf", false);
        }
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
    
    // TF components
    std::shared_ptr<tf2_ros::Buffer> monica_tf_buffer_;
    std::shared_ptr<tf2_ros::Buffer> ross_tf_buffer_;
    
    // TF subscribers for custom topics
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr monica_tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ross_tf_sub_;
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    
    // Last successful transform times for safety
    rclcpp::Time last_monica_transform_time_;
    rclcpp::Time last_ross_transform_time_;
    bool transforms_initialized_;
    
    bool getTransform(std::shared_ptr<tf2_ros::Buffer> buffer, 
                     const std::string& target_frame, 
                     const std::string& source_frame,
                     geometry_msgs::msg::TransformStamped& transform)
    {
        try {
            transform = buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "TF Exception for %s->%s: %s", 
                        source_frame.c_str(), target_frame.c_str(), ex.what());
            return false;
        }
    }
    
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& quaternion)
    {
        tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        return tf2::getYaw(q);
    }
    
    std::pair<double, double> calculateTargetPosition(const geometry_msgs::msg::TransformStamped& monica_transform)
    {
        double monica_yaw = quaternionToYaw(monica_transform.transform.rotation);
        
        // Calculate target position 0.5m behind Monica
        double target_x = monica_transform.transform.translation.x - following_distance_ * std::cos(monica_yaw);
        double target_y = monica_transform.transform.translation.y - following_distance_ * std::sin(monica_yaw);
        
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
        geometry_msgs::msg::TransformStamped monica_transform, ross_transform;
        
        // Get transforms from base_link to map for both robots
        bool monica_ok = getTransform(monica_tf_buffer_, "map", "base_link", monica_transform);
        bool ross_ok = getTransform(ross_tf_buffer_, "map", "base_link", ross_transform);
        
        if (!monica_ok || !ross_ok) {
            if (!monica_ok) {
                RCLCPP_DEBUG(this->get_logger(), "Failed to get Monica's transform (base_link->map)");
            }
            if (!ross_ok) {
                RCLCPP_DEBUG(this->get_logger(), "Failed to get Ross's transform (base_link->map)");
            }
            return;
        }
        
        // Update last successful transform times
        last_monica_transform_time_ = this->now();
        last_ross_transform_time_ = this->now();
        transforms_initialized_ = true;
        
        // Calculate target position
        auto [target_x, target_y] = calculateTargetPosition(monica_transform);
        
        // Current Ross position
        double ross_x = ross_transform.transform.translation.x;
        double ross_y = ross_transform.transform.translation.y;
        double ross_yaw = quaternionToYaw(ross_transform.transform.rotation);
        
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
            "Monica pos: (%.3f, %.3f), Ross pos: (%.3f, %.3f), "
            "Target: (%.3f, %.3f), Distance error: %.3f m, Angle error: %.3f rad, "
            "Cmd: linear=%.3f, angular=%.3f",
            monica_transform.transform.translation.x, monica_transform.transform.translation.y,
            ross_x, ross_y, target_x, target_y,
            distance_error, angle_error, cmd_vel.linear.x, cmd_vel.angular.z);
    }
    
    void safetyCheck()
    {
        if (!transforms_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for TF transforms...");
            stopRobot();
            return;
        }
        
        rclcpp::Time current_time = this->now();
        rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(2.0);
        
        if ((current_time - last_monica_transform_time_) > timeout_duration ||
            (current_time - last_ross_transform_time_) > timeout_duration) {
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF transforms are stale, stopping robot for safety");
            stopRobot();
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