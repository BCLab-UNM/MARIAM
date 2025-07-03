#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class AgentDisplacementEstimator : public rclcpp::Node
{
public:
    AgentDisplacementEstimator() : Node("agent_displacement_estimator")
    {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create publisher for agent displacement
        displacement_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "agent_displacement", 10);
        
        // Create timer to publish at 500 Hz
        timer_ = this->create_wall_timer(
            2ms, std::bind(&AgentDisplacementEstimator::publish_displacement, this));
        
        RCLCPP_INFO(this->get_logger(), "Agent Displacement Estimator node started");
    }

private:
    void publish_displacement()
    {
        try
        {
            // Get transforms from base_link to odom for both agents
            geometry_msgs::msg::TransformStamped monica_transform;
            geometry_msgs::msg::TransformStamped ross_transform;
            
            // Get current time
            rclcpp::Time now = this->get_clock()->now();
            
            // Lookup transforms with timeout
            monica_transform = tf_buffer_->lookupTransform(
                "monica/odom", "monica/base_link", now, rclcpp::Duration::from_nanoseconds(10000000)); // 10ms timeout
            
            ross_transform = tf_buffer_->lookupTransform(
                "ross/odom", "ross/base_link", now, rclcpp::Duration::from_nanoseconds(10000000)); // 10ms timeout
            
            // Extract positions
            double monica_x = monica_transform.transform.translation.x;
            double monica_y = monica_transform.transform.translation.y;
            double monica_z = monica_transform.transform.translation.z;
            
            double ross_x = ross_transform.transform.translation.x;
            double ross_y = ross_transform.transform.translation.y;
            double ross_z = ross_transform.transform.translation.z;
            
            // Calculate Euclidean distance
            double distance = std::sqrt(
                std::pow(monica_x - ross_x, 2) + 
                std::pow(monica_y - ross_y, 2) + 
                std::pow(monica_z - ross_z, 2)
            );
            
            // Publish the distance
            auto message = std_msgs::msg::Float64();
            message.data = distance;
            displacement_publisher_->publish(message);
            
        }
        catch (tf2::TransformException &ex)
        {
            // Skip publishing if transforms are not available
            RCLCPP_DEBUG(this->get_logger(), "Could not get transforms: %s", ex.what());
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr displacement_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentDisplacementEstimator>());
    rclcpp::shutdown();
    return 0;
}