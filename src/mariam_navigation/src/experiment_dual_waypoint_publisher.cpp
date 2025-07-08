#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/exceptions.h>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class ExperimentDualWaypointPublisher : public rclcpp::Node
{
public:
    ExperimentDualWaypointPublisher() : Node("experiment_waypoint_publisher")
    {
        // Hard-coded values
        ross_namespace_ = "ross";
        monica_namespace_ = "monica";
        goal_delay_ = 10.0;
        ross_x_offset_ = 1.0;
        monica_x_offset_ = -1.0;
        
        // TF2 setup - separate buffers for each robot's namespaced topics
        ross_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        monica_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        
        // Subscribe to namespaced TF topics
        ross_tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/ross/tf", 10, 
            std::bind(&ExperimentDualWaypointPublisher::ross_tf_callback, this, std::placeholders::_1));
            
        monica_tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/monica/tf", 10, 
            std::bind(&ExperimentDualWaypointPublisher::monica_tf_callback, this, std::placeholders::_1));
        
        // Goal pose publishers
        ross_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ross/goal_pose", 10);
        
        monica_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/monica/goal_pose", 10);
        
        // Timer to trigger goals after specified delay
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(goal_delay_),
            std::bind(&ExperimentDualWaypointPublisher::send_goals, this));
        
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
    
    // TF2 - separate buffers for each robot
    std::shared_ptr<tf2_ros::Buffer> ross_tf_buffer_;
    std::shared_ptr<tf2_ros::Buffer> monica_tf_buffer_;
    
    // TF subscribers for namespaced topics
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ross_tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr monica_tf_sub_;
    
    // Goal pose publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ross_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr monica_goal_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    void ross_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            ross_tf_buffer_->setTransform(transform, "ross_tf");
        }
    }
    
    void monica_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            monica_tf_buffer_->setTransform(transform, "monica_tf");
        }
    }
    
    geometry_msgs::msg::PoseStamped get_robot_pose(const std::string& robot_namespace)
    {
        geometry_msgs::msg::PoseStamped pose;
        
        // Select the correct TF buffer based on robot namespace
        std::shared_ptr<tf2_ros::Buffer> buffer;
        if (robot_namespace == "ross") {
            buffer = ross_tf_buffer_;
        } else if (robot_namespace == "monica") {
            buffer = monica_tf_buffer_;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown robot namespace: %s", robot_namespace.c_str());
            pose.header.frame_id = ""; // Mark as invalid
            return pose;
        }
        
        try {
            // Get transform from map to robot's base_footprint
            geometry_msgs::msg::TransformStamped transform = buffer->lookupTransform(
                "map", 
                "base_footprint",
                tf2::TimePointZero);
            
            // Create a pose at origin in robot's frame
            geometry_msgs::msg::PoseStamped robot_pose;
            robot_pose.header.frame_id = "base_footprint";
            robot_pose.header.stamp = this->get_clock()->now();
            robot_pose.pose.position.x = 0.0;
            robot_pose.pose.position.y = 0.0;
            robot_pose.pose.position.z = 0.0;
            robot_pose.pose.orientation.w = 1.0;
            
            // Transform to map frame
            tf2::doTransform(robot_pose, pose, transform);
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), 
                "Could not get pose for %s: %s", robot_namespace.c_str(), ex.what());
            pose.header.frame_id = ""; // Mark as invalid
        }
        
        return pose;
    }
    
    geometry_msgs::msg::PoseStamped create_goal_pose(
        const geometry_msgs::msg::PoseStamped& current_pose, 
        double x_offset)
    {
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->get_clock()->now();
        
        // Position: add offset to current position
        goal_pose.pose.position.x = current_pose.pose.position.x + x_offset;
        goal_pose.pose.position.y = current_pose.pose.position.y;
        goal_pose.pose.position.z = current_pose.pose.position.z;
        
        // Orientation: facing +x direction (yaw = 0)
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;
        
        return goal_pose;
    }
    
    void send_goals()
    {
        RCLCPP_INFO(this->get_logger(), "Sending navigation goals to robots...");
        
        // Get current poses
        auto ross_pose = get_robot_pose(ross_namespace_);
        auto monica_pose = get_robot_pose(monica_namespace_);
        
        if (ross_pose.header.frame_id.empty() || monica_pose.header.frame_id.empty()) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to get robot poses. Retrying in 5 seconds...");
            timer_ = this->create_wall_timer(
                5s, std::bind(&ExperimentDualWaypointPublisher::send_goals, this));
            return;
        }
        
        // Create goal poses
        auto ross_goal = create_goal_pose(ross_pose, ross_x_offset_);
        auto monica_goal = create_goal_pose(monica_pose, monica_x_offset_);
        
        // Publish goals
        RCLCPP_INFO(this->get_logger(), 
            "Publishing %s goal to: x=%.2f, y=%.2f", 
            ross_namespace_.c_str(), ross_goal.pose.position.x, ross_goal.pose.position.y);
        RCLCPP_INFO(this->get_logger(), 
            "Publishing %s goal to: x=%.2f, y=%.2f", 
            monica_namespace_.c_str(), monica_goal.pose.position.x, monica_goal.pose.position.y);
        
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
    auto node = std::make_shared<ExperimentDualWaypointPublisher>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}