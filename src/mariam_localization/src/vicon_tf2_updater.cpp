#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

class ViconTF2Updater : public rclcpp::Node
{
public:
    ViconTF2Updater() : Node("vicon_tf2_updater")
    {
        // initialize broadcaster to publish to TF tree
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::string topic_name;

        if (this->get_namespace() == "/ross") {
          topic_name = "/world_ross_pose";
        }
        else {
          topic_name = "/world_monica_pose";
        }
        
        world_to_robot_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            topic_name,
            10,
            std::bind(&ViconTF2Updater::sub_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Vicon TF2 updater node started");
    }

private:
    void sub_callback(const geometry_msgs::msg::Pose & msg) {
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "vicon_link";

        t.transform.translation.x = msg.position.x;
        t.transform.translation.y = msg.position.y;
        t.transform.translation.z = msg.position.z;

        t.transform.rotation.x = msg.orientation.x;
        t.transform.rotation.y = msg.orientation.y;
        t.transform.rotation.z = msg.orientation.z;
        t.transform.rotation.w = msg.orientation.w;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr world_to_robot_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ViconTF2Updater>());
    rclcpp::shutdown();
    return 0;
}