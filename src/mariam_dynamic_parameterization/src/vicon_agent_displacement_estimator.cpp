#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

class DistanceCalculatorNode : public rclcpp::Node
{
public:
    DistanceCalculatorNode() : Node("distance_calculator_node")
    {
        // Initialize pose variables
        monica_pose_received_ = false;
        ross_pose_received_ = false;

        // Create subscribers
        monica_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_monica_pose", 
            10, 
            std::bind(&DistanceCalculatorNode::monica_pose_callback, this, std::placeholders::_1)
        );

        ross_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_ross_pose", 
            10, 
            std::bind(&DistanceCalculatorNode::ross_pose_callback, this, std::placeholders::_1)
        );

        // Create publisher
        distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "agent_displacement", 
            10
        );

        RCLCPP_INFO(this->get_logger(), "Distance Calculator Node initialized");
    }

private:
    void monica_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        monica_pose_ = *msg;
        monica_pose_received_ = true;
        calculate_and_publish_distance();
    }

    void ross_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        ross_pose_ = *msg;
        ross_pose_received_ = true;
        calculate_and_publish_distance();
    }

    void calculate_and_publish_distance()
    {
        // Only calculate if we have received both poses
        if (!monica_pose_received_ || !ross_pose_received_)
        {
            return;
        }

        // Calculate 3D Euclidean distance between the positions
        double dx = monica_pose_.position.x - ross_pose_.position.x;
        double dy = monica_pose_.position.y - ross_pose_.position.y;
        double dz = monica_pose_.position.z - ross_pose_.position.z;

        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        // Publish the distance
        auto distance_msg = std_msgs::msg::Float64();
        distance_msg.data = distance;
        distance_publisher_->publish(distance_msg);

        RCLCPP_DEBUG(this->get_logger(), 
            "Published distance: %.3f (Monica: [%.3f, %.3f, %.3f], Ross: [%.3f, %.3f, %.3f])", 
            distance,
            monica_pose_.position.x, monica_pose_.position.y, monica_pose_.position.z,
            ross_pose_.position.x, ross_pose_.position.y, ross_pose_.position.z
        );
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr monica_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ross_subscriber_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher_;

    // Pose storage
    geometry_msgs::msg::Pose monica_pose_;
    geometry_msgs::msg::Pose ross_pose_;
    bool monica_pose_received_;
    bool ross_pose_received_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceCalculatorNode>());
    rclcpp::shutdown();
    return 0;
}