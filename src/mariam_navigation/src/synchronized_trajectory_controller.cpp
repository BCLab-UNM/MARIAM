#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <vector>
#include <algorithm>
#include <cmath>

class SynchronizedTrajectoryController : public rclcpp::Node
{
public:
    SynchronizedTrajectoryController() : Node("synchronized_trajectory_controller"),
        v_max_(0.2), a_max_(0.1), control_frequency_(50.0),
        trajectory_started_(false), trajectory_complete_(false)
    {
        // Publishers for cmd_vel
        monica_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/monica/cmd_vel", 10);
        ross_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/ross/cmd_vel", 10);
        
        // Subscribers for current poses
        monica_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_monica_pose", 10,
            std::bind(&SynchronizedTrajectoryController::monicaPoseCallback, this, std::placeholders::_1));
        
        ross_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_ross_pose", 10,
            std::bind(&SynchronizedTrajectoryController::rossPoseCallback, this, std::placeholders::_1));
        
        // Subscribers for paths from your Hermite spline planner
        monica_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/monica_path", 10,
            std::bind(&SynchronizedTrajectoryController::monicaPathCallback, this, std::placeholders::_1));
        
        ross_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/ross_path", 10,
            std::bind(&SynchronizedTrajectoryController::rossPathCallback, this, std::placeholders::_1));
        
        // Control timer at 50 Hz
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            std::bind(&SynchronizedTrajectoryController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Synchronized Trajectory Controller initialized. Waiting for paths and poses...");
    }

private:
    double getYawFromQuaternion(const tf2::Quaternion& q)
    {
        tf2::Matrix3x3 matrix(q);
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        return yaw;
    }
        geometry_msgs::msg::Pose pose;
        double timestamp;
        geometry_msgs::msg::Twist expected_velocity;
    };

    struct TimedWaypoint {
    double v_max_;
    double a_max_;
    double control_frequency_;
    
    // State tracking
    bool trajectory_started_;
    bool trajectory_complete_;
    rclcpp::Time trajectory_start_time_;
    bool monica_pose_received_ = false;
    bool ross_pose_received_ = false;
    bool monica_path_received_ = false;
    bool ross_path_received_ = false;
    
    // Current poses
    geometry_msgs::msg::Pose monica_current_pose_;
    geometry_msgs::msg::Pose ross_current_pose_;
    
    // Trajectories
    std::vector<TimedWaypoint> monica_trajectory_;
    std::vector<TimedWaypoint> ross_trajectory_;
    
    // ROS2 components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr monica_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ross_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr monica_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ross_path_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void monicaPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        monica_current_pose_ = *msg;
        monica_pose_received_ = true;
    }

    void rossPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        ross_current_pose_ = *msg;
        ross_pose_received_ = true;
    }

    void monicaPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Monica path with %zu poses", msg->poses.size());
        monica_trajectory_ = createTimedTrajectory(msg->poses);
        monica_path_received_ = true;
        checkAndStartTrajectory();
    }

    void rossPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Ross path with %zu poses", msg->poses.size());
        ross_trajectory_ = createTimedTrajectory(msg->poses);
        ross_path_received_ = true;
        checkAndStartTrajectory();
    }

    double calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
    {
        double dx = pose2.position.x - pose1.position.x;
        double dy = pose2.position.y - pose1.position.y;
        double dz = pose2.position.z - pose1.position.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    double calculatePathLength(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
    {
        if (poses.size() < 2) return 0.0;
        
        double total_length = 0.0;
        for (size_t i = 1; i < poses.size(); i++) {
            total_length += calculateDistance(poses[i-1].pose, poses[i].pose);
        }
        return total_length;
    }

    std::vector<TimedWaypoint> createTimedTrajectory(const std::vector<geometry_msgs::msg::PoseStamped>& path_poses)
    {
        std::vector<TimedWaypoint> timed_trajectory;
        if (path_poses.empty()) return timed_trajectory;

        // Calculate total path length
        double total_length = calculatePathLength(path_poses);
        
        // Use synchronized timing: both robots finish at the same time
        // Time = distance / average_speed, considering acceleration phase
        double avg_speed = v_max_ * 0.8; // Conservative estimate accounting for acceleration
        double total_time = total_length / avg_speed;
        
        // Create timed waypoints
        double current_time = 0.0;
        double cumulative_distance = 0.0;
        
        for (size_t i = 0; i < path_poses.size(); i++) {
            TimedWaypoint timed_point;
            timed_point.pose = path_poses[i].pose;
            
            if (i > 0) {
                double segment_distance = calculateDistance(path_poses[i-1].pose, path_poses[i].pose);
                cumulative_distance += segment_distance;
                current_time = (cumulative_distance / total_length) * total_time;
            }
            
            timed_point.timestamp = current_time;
            
            // Calculate expected velocity (simplified - constant speed for now)
            timed_point.expected_velocity = calculateExpectedVelocity(path_poses, i, avg_speed);
            
            timed_trajectory.push_back(timed_point);
        }
        
        RCLCPP_INFO(this->get_logger(), "Created timed trajectory: length=%.2fm, time=%.2fs, points=%zu", 
                    total_length, total_time, timed_trajectory.size());
        
        return timed_trajectory;
    }

    geometry_msgs::msg::Twist calculateExpectedVelocity(
        const std::vector<geometry_msgs::msg::PoseStamped>& path_poses, 
        size_t index, double speed)
    {
        geometry_msgs::msg::Twist velocity;
        
        if (index + 1 < path_poses.size()) {
            // Calculate direction to next waypoint
            double dx = path_poses[index + 1].pose.position.x - path_poses[index].pose.position.x;
            double dy = path_poses[index + 1].pose.position.y - path_poses[index].pose.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance > 1e-6) {
                velocity.linear.x = (dx / distance) * speed;
                velocity.linear.y = (dy / distance) * speed;
                
                // Calculate angular velocity from orientation change
                tf2::Quaternion q1, q2;
                tf2::fromMsg(path_poses[index].pose.orientation, q1);
                tf2::fromMsg(path_poses[index + 1].pose.orientation, q2);
                
                double yaw1 = getYawFromQuaternion(q1);
                double yaw2 = getYawFromQuaternion(q2);
                double dyaw = yaw2 - yaw1;
                
                // Normalize angle difference
                while (dyaw > M_PI) dyaw -= 2*M_PI;
                while (dyaw < -M_PI) dyaw += 2*M_PI;
                
                velocity.angular.z = dyaw * speed / distance; // Rough approximation
            }
        }
        
        return velocity;
    }

    void checkAndStartTrajectory()
    {
        if (monica_path_received_ && ross_path_received_ && 
            monica_pose_received_ && ross_pose_received_ && !trajectory_started_) {
            
            trajectory_start_time_ = this->get_clock()->now();
            trajectory_started_ = true;
            trajectory_complete_ = false;
            
            RCLCPP_INFO(this->get_logger(), "Starting synchronized trajectory execution!");
        }
    }

    TimedWaypoint interpolateTrajectory(const std::vector<TimedWaypoint>& trajectory, double current_time)
    {
        if (trajectory.empty()) {
            TimedWaypoint empty;
            return empty;
        }
        
        if (current_time <= trajectory.front().timestamp) {
            return trajectory.front();
        }
        
        if (current_time >= trajectory.back().timestamp) {
            return trajectory.back();
        }
        
        // Find the two waypoints to interpolate between
        for (size_t i = 1; i < trajectory.size(); i++) {
            if (current_time <= trajectory[i].timestamp) {
                double t = (current_time - trajectory[i-1].timestamp) / 
                          (trajectory[i].timestamp - trajectory[i-1].timestamp);
                
                TimedWaypoint interpolated;
                interpolated.timestamp = current_time;
                
                // Linear interpolation of position
                interpolated.pose.position.x = trajectory[i-1].pose.position.x + 
                    t * (trajectory[i].pose.position.x - trajectory[i-1].pose.position.x);
                interpolated.pose.position.y = trajectory[i-1].pose.position.y + 
                    t * (trajectory[i].pose.position.y - trajectory[i-1].pose.position.y);
                interpolated.pose.position.z = trajectory[i-1].pose.position.z + 
                    t * (trajectory[i].pose.position.z - trajectory[i-1].pose.position.z);
                
                // SLERP for orientation
                tf2::Quaternion q1, q2;
                tf2::fromMsg(trajectory[i-1].pose.orientation, q1);
                tf2::fromMsg(trajectory[i].pose.orientation, q2);
                tf2::Quaternion q_interp = q1.slerp(q2, t);
                interpolated.pose.orientation = tf2::toMsg(q_interp);
                
                // Interpolate expected velocity
                interpolated.expected_velocity.linear.x = trajectory[i-1].expected_velocity.linear.x + 
                    t * (trajectory[i].expected_velocity.linear.x - trajectory[i-1].expected_velocity.linear.x);
                interpolated.expected_velocity.linear.y = trajectory[i-1].expected_velocity.linear.y + 
                    t * (trajectory[i].expected_velocity.linear.y - trajectory[i-1].expected_velocity.linear.y);
                interpolated.expected_velocity.angular.z = trajectory[i-1].expected_velocity.angular.z + 
                    t * (trajectory[i].expected_velocity.angular.z - trajectory[i-1].expected_velocity.angular.z);
                
                return interpolated;
            }
        }
        
        return trajectory.back();
    }

    geometry_msgs::msg::Twist calculateCmdVel(const geometry_msgs::msg::Pose& current_pose, 
                                             const TimedWaypoint& target_waypoint)
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        // Position errors
        double error_x = target_waypoint.pose.position.x - current_pose.position.x;
        double error_y = target_waypoint.pose.position.y - current_pose.position.y;
        
        // Orientation error
        tf2::Quaternion q_current, q_target;
        tf2::fromMsg(current_pose.orientation, q_current);
        tf2::fromMsg(target_waypoint.pose.orientation, q_target);
        
        double yaw_current = getYawFromQuaternion(q_current);
        double yaw_target = getYawFromQuaternion(q_target);
        double error_yaw = yaw_target - yaw_current;
        
        // Normalize angle error
        while (error_yaw > M_PI) error_yaw -= 2*M_PI;
        while (error_yaw < -M_PI) error_yaw += 2*M_PI;
        
        // PID gains (tune these as needed)
        double kp_linear = 2.0;
        double kp_angular = 3.0;
        
        // Feedforward + feedback control
        cmd_vel.linear.x = target_waypoint.expected_velocity.linear.x + kp_linear * error_x;
        cmd_vel.linear.y = target_waypoint.expected_velocity.linear.y + kp_linear * error_y;
        cmd_vel.angular.z = target_waypoint.expected_velocity.angular.z + kp_angular * error_yaw;
        
        // Apply velocity limits
        double linear_mag = std::sqrt(cmd_vel.linear.x*cmd_vel.linear.x + cmd_vel.linear.y*cmd_vel.linear.y);
        if (linear_mag > v_max_) {
            cmd_vel.linear.x *= v_max_ / linear_mag;
            cmd_vel.linear.y *= v_max_ / linear_mag;
        }
        
        cmd_vel.angular.z = std::max(-2.0, std::min(2.0, cmd_vel.angular.z)); // Limit angular velocity
        
        return cmd_vel;
    }

    void controlLoop()
    {
        if (!trajectory_started_ || trajectory_complete_) {
            // Send zero velocity when not executing trajectory
            geometry_msgs::msg::Twist zero_cmd;
            monica_cmd_pub_->publish(zero_cmd);
            ross_cmd_pub_->publish(zero_cmd);
            return;
        }
        
        if (!monica_pose_received_ || !ross_pose_received_) {
            return; // Wait for pose updates
        }
        
        // Calculate current time from trajectory start
        double current_time = (this->get_clock()->now() - trajectory_start_time_).seconds();
        
        // Get target waypoints for current time
        TimedWaypoint monica_target = interpolateTrajectory(monica_trajectory_, current_time);
        TimedWaypoint ross_target = interpolateTrajectory(ross_trajectory_, current_time);
        
        // Calculate cmd_vel for both robots
        geometry_msgs::msg::Twist monica_cmd = calculateCmdVel(monica_current_pose_, monica_target);
        geometry_msgs::msg::Twist ross_cmd = calculateCmdVel(ross_current_pose_, ross_target);
        
        // Publish synchronized commands
        monica_cmd_pub_->publish(monica_cmd);
        ross_cmd_pub_->publish(ross_cmd);
        
        // Check if trajectory is complete
        if (!monica_trajectory_.empty() && !ross_trajectory_.empty()) {
            double monica_end_time = monica_trajectory_.back().timestamp;
            double ross_end_time = ross_trajectory_.back().timestamp;
            double max_end_time = std::max(monica_end_time, ross_end_time);
            
            if (current_time > max_end_time + 1.0) { // 1 second buffer
                trajectory_complete_ = true;
                RCLCPP_INFO(this->get_logger(), "Trajectory execution complete!");
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SynchronizedTrajectoryController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}