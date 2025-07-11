#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <memory>

class DisplacementPublisher : public rclcpp::Node
{
public:
    DisplacementPublisher() : Node("displacement_publisher")
    {
        // Initialize subscribers
        monica_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "monica/wheel/odom", 10,
            std::bind(&DisplacementPublisher::monica_callback, this, std::placeholders::_1));
        
        ross_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ross/wheel/odom", 10,
            std::bind(&DisplacementPublisher::ross_callback, this, std::placeholders::_1));
        
        // Initialize publisher
        displacement_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/robot_displacement_error", 10);
        
        // Initialize flags
        monica_received_ = false;
        ross_received_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Displacement Publisher Node Started");
    }

private:
    void monica_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        monica_position_ = msg->pose.pose.position;
        monica_position_.x = -monica_position_.x; // Negate x-coordinate since Monica drives backwards
        monica_position_.y = -monica_position_.y; // Negate y-coordinate since Monica drives backwards
        monica_received_ = true;
        
        // Calculate and publish displacement if we have both messages
        if (ross_received_) {
            calculate_and_publish_displacement();
        }
    }
    
    void ross_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ross_position_ = msg->pose.pose.position;
        ross_received_ = true;
        
        // Calculate and publish displacement if we have both messages
        if (monica_received_) {
            calculate_and_publish_displacement();
        }
    }
    
    void calculate_and_publish_displacement()
    {
        // Calculate 3D distance magnitude
        double dx = monica_position_.x - ross_position_.x;
        double dy = monica_position_.y - ross_position_.y;
        double dz = monica_position_.z - ross_position_.z;
        
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // Create and publish message
        auto displacement_msg = std_msgs::msg::Float64();
        displacement_msg.data = distance;
        
        displacement_pub_->publish(displacement_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Published displacement: %.3f (Monica: [%.3f, %.3f, %.3f], Ross: [%.3f, %.3f, %.3f])",
                    distance, 
                    monica_position_.x, monica_position_.y, monica_position_.z,
                    ross_position_.x, ross_position_.y, ross_position_.z);
    }
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr monica_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ross_sub_;
    
    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr displacement_pub_;
    
    // Store latest positions
    geometry_msgs::msg::Point monica_position_;
    geometry_msgs::msg::Point ross_position_;
    
    // Flags to track if we've received messages
    bool monica_received_;
    bool ross_received_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplacementPublisher>());
    rclcpp::shutdown();
    return 0;
}