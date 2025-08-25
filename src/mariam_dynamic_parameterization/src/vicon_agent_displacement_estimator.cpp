#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

class DistanceCalculatorNode : public rclcpp::Node
{
public:
    DistanceCalculatorNode() : Node("distance_calculator_node")
    {
        // Initialize pose variables
        monica_pose_received_ = false;
        ross_pose_received_ = false;

        // change QoS settings
        auto qos_profile = rclcpp::QoS(10);
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        // Create subscribers
        monica_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_monica_pose", 
            qos_profile, 
            std::bind(&DistanceCalculatorNode::monica_pose_callback, this, std::placeholders::_1)
        );

        ross_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_ross_pose", 
            qos_profile, 
            std::bind(&DistanceCalculatorNode::ross_pose_callback, this, std::placeholders::_1)
        );

        // Create publisher
        distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "agent_displacement", 
            qos_profile
        );

        RCLCPP_INFO(this->get_logger(), "Distance Calculator Node initialized");
    }

private:
    void monica_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        // T_vm transform (vicon to manipulator) - hardcoded
        tf2::Transform T_vm;
        T_vm.setOrigin(tf2::Vector3(0.23, -0.07, -0.06));
        T_vm.setBasis(tf2::Matrix3x3(0, 1, 0, -1, 0, 0, 0, 0, 1));
        
        // Convert incoming T_wv pose to tf2::Transform
        tf2::Transform T_wv;
        tf2::fromMsg(*msg, T_wv);
        
        // Chain transformations: T_wm = T_wv * T_vm
        tf2::Transform T_wm = T_wv * T_vm;
        
        // Convert back to pose and store
        tf2::toMsg(T_wm, monica_manipulator_pose_);
        monica_pose_received_ = true;
        calculate_and_publish_distance();
    }

    void ross_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        // T_vm transform (vicon to manipulator) - hardcoded
        tf2::Transform T_vm;
        T_vm.setOrigin(tf2::Vector3(0.23, -0.07, -0.06));
        T_vm.setBasis(tf2::Matrix3x3(0, 1, 0, -1, 0, 0, 0, 0, 1));
        
        // Convert incoming T_wv pose to tf2::Transform
        tf2::Transform T_wv;
        tf2::fromMsg(*msg, T_wv);
        
        // Chain transformations: T_wm = T_wv * T_vm
        tf2::Transform T_wm = T_wv * T_vm;
        
        // Convert back to pose and store
        tf2::toMsg(T_wm, ross_manipulator_pose_);
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
        double dx = monica_manipulator_pose_.position.x - ross_manipulator_pose_.position.x;
        double dy = monica_manipulator_pose_.position.y - ross_manipulator_pose_.position.y;
        double dz = monica_manipulator_pose_.position.z - ross_manipulator_pose_.position.z;

        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        // Publish the distance
        auto distance_msg = std_msgs::msg::Float64();
        distance_msg.data = distance;
        distance_publisher_->publish(distance_msg);

        RCLCPP_DEBUG(this->get_logger(), 
            "Published distance: %.3f (Monica: [%.3f, %.3f, %.3f], Ross: [%.3f, %.3f, %.3f])", 
            distance,
            monica_manipulator_pose_.position.x, monica_manipulator_pose_.position.y, monica_manipulator_pose_.position.z,
            ross_manipulator_pose_.position.x, ross_manipulator_pose_.position.y, ross_manipulator_pose_.position.z
        );
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr monica_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ross_subscriber_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher_;

    // Pose storage
    geometry_msgs::msg::Pose monica_manipulator_pose_;
    geometry_msgs::msg::Pose ross_manipulator_pose_;
    bool monica_pose_received_;
    bool ross_pose_received_;

    // Transform vicon to manipulator
    const double T_vm[4][4] = {
    {0,  1, 0,  0.23},
    {-1, 0, 0, -0.07},
    {0,  0, 1, -0.06},
    {0,  0, 0,  1}
};

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceCalculatorNode>());
    rclcpp::shutdown();
    return 0;
}