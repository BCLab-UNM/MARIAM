#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class VicomPosePublisher : public rclcpp::Node
{
public:
    VicomPosePublisher() : Node("vicom_pose_publisher")
    {
        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create publishers
        ross_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("world_ross_pose", 10);
        monica_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("world_monica_pose", 10);
        
        // Create timer for 1000 Hz (1ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&VicomPosePublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Vicom Pose Publisher node started");
    }

private:
    void timer_callback()
    {
        try {
            // Look up world -> ross transformation
            auto ross_transform = tf_buffer_->lookupTransform("world", "ross", tf2::TimePointZero);
            geometry_msgs::msg::Pose ross_pose;
            ross_pose.position.x = ross_transform.transform.translation.x;
            ross_pose.position.y = ross_transform.transform.translation.y;
            ross_pose.position.z = ross_transform.transform.translation.z;
            ross_pose.orientation = ross_transform.transform.rotation;
            ross_pub_->publish(ross_pose);
        }
        catch (tf2::TransformException &ex) {
            // Skip publishing if transform not available
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not get world->ross transform: %s", ex.what());
        }
        
        try {
            // Look up world -> monica transformation
            auto monica_transform = tf_buffer_->lookupTransform("world", "monica", tf2::TimePointZero);
            geometry_msgs::msg::Pose monica_pose;
            monica_pose.position.x = monica_transform.transform.translation.x;
            monica_pose.position.y = monica_transform.transform.translation.y;
            monica_pose.position.z = monica_transform.transform.translation.z;
            monica_pose.orientation = monica_transform.transform.rotation;
            monica_pub_->publish(monica_pose);
        }
        catch (tf2::TransformException &ex) {
            // Skip publishing if transform not available
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not get world->monica transform: %s", ex.what());
        }
    }
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ross_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr monica_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VicomPosePublisher>());
    rclcpp::shutdown();
    return 0;
}