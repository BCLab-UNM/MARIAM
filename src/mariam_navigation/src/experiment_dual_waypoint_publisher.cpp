#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class ExperimentWaypointPublisher : public rclcpp::Node
{
public:
    ExperimentWaypointPublisher() : Node("experiment_waypoint_publisher")
    {
        // Hard-coded values
        ross_namespace_ = "ross";
        monica_namespace_ = "monica";
        goal_delay_ = 10.0;
        ross_x_offset_ = 2.0;
        monica_x_offset_ = -2.0;
        
        // Initialize pose validity flags
        ross_pose_received_ = false;
        monica_pose_received_ = false;
        
        // Subscribe to robot pose topics
        ross_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ross/pose", 10, 
            std::bind(&ExperimentWaypointPublisher::ross_pose_callback, this, std::placeholders::_1));
            
        monica_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/monica/pose", 10, 
            std::bind(&ExperimentWaypointPublisher::monica_pose_callback, this, std::placeholders::_1));
        
        // Goal pose publishers
        ross_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ross/goal_pose", 10);
        
        monica_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/monica/goal_pose", 10);
        
        // Timer to trigger goals after specified delay
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(goal_delay_),
            std::bind(&ExperimentWaypointPublisher::send_goals, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Experiment waypoint publisher initialized. Goals will be sent in %.1f seconds...", 
            goal_delay_);
        RCLCPP_INFO(this->get_logger(), 
            "Robot namespaces: %s (offset: %.2fm), %s (offset: %.2fm)", 
            ross_namespace_.c_str(), ross_x_offset_, 
            monica_namespace_.c_str(), monica_x_offset_);
    }

private:
    // Hard-coded values
    std::string ross_namespace_;
    std::string monica_namespace_;
    double goal_delay_;
    double ross_x_offset_;
    double monica_x_offset_;
    
    // Current robot poses
    geometry_msgs::msg::PoseWithCovarianceStamped ross_current_pose_;
    geometry_msgs::msg::PoseWithCovarianceStamped monica_current_pose_;
    bool ross_pose_received_;
    bool monica_pose_received_;
    
    // Pose subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ross_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr monica_pose_sub_;
    
    // Goal pose publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ross_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr monica_goal_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    void ross_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        ross_current_pose_ = *msg;
        ross_pose_received_ = true;
        RCLCPP_DEBUG(this->get_logger(), "Received Ross pose: x=%.2f, y=%.2f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);
    }
    
    void monica_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        monica_current_pose_ = *msg;
        monica_pose_received_ = true;
        RCLCPP_DEBUG(this->get_logger(), "Received Monica pose: x=%.2f, y=%.2f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);
    }
    
    geometry_msgs::msg::PoseStamped create_goal_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped& current_pose, 
        double x_offset)
    {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = current_pose.header.frame_id; // Should be "map"
        goal_pose.header.stamp = this->get_clock()->now();
        
        // Position: add offset to current position
        goal_pose.pose.position.x = current_pose.pose.pose.position.x + x_offset;
        goal_pose.pose.position.y = current_pose.pose.pose.position.y;
        goal_pose.pose.position.z = current_pose.pose.pose.position.z;
        
        // Orientation: facing +x direction (yaw = 0)
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;
        
        return goal_pose;
    }
    
    void send_goals()
    {
        RCLCPP_INFO(this->get_logger(), "Attempting to send navigation goals to robots...");
        
        // Check if we have received poses from both robots
        if (!ross_pose_received_ || !monica_pose_received_) {
            RCLCPP_WARN(this->get_logger(), 
                "Haven't received poses from all robots yet (Ross: %s, Monica: %s). Retrying in 5 seconds...",
                ross_pose_received_ ? "✓" : "✗", monica_pose_received_ ? "✓" : "✗");
            timer_ = this->create_wall_timer(
                5s, std::bind(&ExperimentWaypointPublisher::send_goals, this));
            return;
        }
        
        // Create goal poses
        auto ross_goal = create_goal_pose(ross_current_pose_, ross_x_offset_);
        auto monica_goal = create_goal_pose(monica_current_pose_, monica_x_offset_);
        
        // Publish goals
        RCLCPP_INFO(this->get_logger(), 
            "Publishing %s goal to: x=%.2f, y=%.2f (from current x=%.2f, y=%.2f)", 
            ross_namespace_.c_str(), 
            ross_goal.pose.position.x, ross_goal.pose.position.y,
            ross_current_pose_.pose.pose.position.x, ross_current_pose_.pose.pose.position.y);
        RCLCPP_INFO(this->get_logger(), 
            "Publishing %s goal to: x=%.2f, y=%.2f (from current x=%.2f, y=%.2f)", 
            monica_namespace_.c_str(), 
            monica_goal.pose.position.x, monica_goal.pose.position.y,
            monica_current_pose_.pose.pose.position.x, monica_current_pose_.pose.pose.position.y);
        
        ross_goal_pub_->publish(ross_goal);
        monica_goal_pub_->publish(monica_goal);
        
        RCLCPP_INFO(this->get_logger(), "Goals published successfully!");
        
        // Cancel the timer so it doesn't repeat
        timer_->cancel();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExperimentWaypointPublisher>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}