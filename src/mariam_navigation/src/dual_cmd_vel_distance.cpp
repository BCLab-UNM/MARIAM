#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>

class DualCmdVelDistance : public rclcpp::Node
{
public:
    DualCmdVelDistance() : Node("dual_cmd_vel_distance")
    {
        // Hardcoded parameters
        distance_ = 2.0;  // meters
        speed_ = 0.1;     // m/s
        
        // Create publishers for both robots
        monica_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("monica/cmd_vel", 10);
        ross_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("ross/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Starting dual robot drive - Distance: %.2f m, Speed: %.2f m/s", 
                    distance_, speed_);
        
        // Use a timer to start driving after the node is fully initialized
        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                start_timer_->cancel();
                drive_robots();
            });
    }

private:
    void drive_robots()
    {
        // Calculate time needed to travel the distance
        double drive_time = distance_ / speed_;
        
        RCLCPP_INFO(this->get_logger(), "Driving for %.2f seconds", drive_time);
        
        // Create velocity messages
        geometry_msgs::msg::Twist monica_cmd;
        geometry_msgs::msg::Twist ross_cmd;
        
        // Monica drives in negative x direction
        monica_cmd.linear.x = -speed_;
        monica_cmd.linear.y = 0.0;
        monica_cmd.linear.z = 0.0;
        monica_cmd.angular.x = 0.0;
        monica_cmd.angular.y = 0.0;
        monica_cmd.angular.z = 0.0;
        
        // Ross drives in positive x direction
        ross_cmd.linear.x = speed_;
        ross_cmd.linear.y = 0.0;
        ross_cmd.linear.z = 0.0;
        ross_cmd.angular.x = 0.0;
        ross_cmd.angular.y = 0.0;
        ross_cmd.angular.z = 0.0;
        
        // Start driving
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::duration<double>(drive_time);
        
        rclcpp::Rate rate(50); // 50 Hz publishing rate
        
        while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time)
        {
            monica_pub_->publish(monica_cmd);
            ross_pub_->publish(ross_cmd);
            
            rate.sleep();
        }
        
        // Stop both robots
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.linear.z = 0.0;
        stop_cmd.angular.x = 0.0;
        stop_cmd.angular.y = 0.0;
        stop_cmd.angular.z = 0.0;
        
        // Send stop commands multiple times to ensure they're received
        for (int i = 0; i < 10; ++i)
        {
            monica_pub_->publish(stop_cmd);
            ross_pub_->publish(stop_cmd);
            rate.sleep();
        }
        
        RCLCPP_INFO(this->get_logger(), "Drive complete - robots stopped");
        
        // Shutdown the node
        rclcpp::shutdown();
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_pub_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    double distance_;
    double speed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DualCmdVelDistance>();
    
    // Keep the node alive until it shuts itself down
    rclcpp::spin(node);
    
    return 0;
}