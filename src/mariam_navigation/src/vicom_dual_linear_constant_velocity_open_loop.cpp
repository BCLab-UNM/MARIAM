#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>

class DualLinearConstantVelocityDistanceBasedVicom : public rclcpp::Node
{
public:
    DualLinearConstantVelocityDistanceBasedVicom() : Node("dual_linear_constant_velocity_distance_based_vicom")
    {
        // Declare parameters with defaults
        this->declare_parameter("distance", 2.0);
        this->declare_parameter("speed", 0.1);
        
        // Get parameters
        distance_ = this->get_parameter("distance").as_double();
        speed_ = this->get_parameter("speed").as_double();
        
        // Initialize state variables
        monica_start_x_ = 0.0;
        monica_start_y_ = 0.0;
        monica_current_x_ = 0.0;
        monica_current_y_ = 0.0;
        monica_distance_traveled_ = 0.0;
        monica_pose_initialized_ = false;
        
        ross_start_x_ = 0.0;
        ross_start_y_ = 0.0;
        ross_current_x_ = 0.0;
        ross_current_y_ = 0.0;
        ross_distance_traveled_ = 0.0;
        ross_pose_initialized_ = false;
        
        // Create publishers
        monica_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("monica/cmd_vel", 10);
        ross_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("ross/cmd_vel", 10);
        
        // Create subscribers
        monica_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_monica_pose", 10,
            std::bind(&DualLinearConstantVelocityDistanceBasedVicom::monica_pose_callback, this, std::placeholders::_1));
        
        ross_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_ross_pose", 10,
            std::bind(&DualLinearConstantVelocityDistanceBasedVicom::ross_pose_callback, this, std::placeholders::_1));
        
        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz control loop
            std::bind(&DualLinearConstantVelocityDistanceBasedVicom::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Starting dual robot distance-based drive (Vicom) - Distance: %.2f m, Speed: %.2f m/s",
                   distance_, speed_);
    }

private:
    void monica_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!monica_pose_initialized_) {
            monica_start_x_ = msg->position.x;
            monica_start_y_ = msg->position.y;
            monica_pose_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Monica pose initialized at (%.3f, %.3f)", 
                       monica_start_x_, monica_start_y_);
        }
        
        monica_current_x_ = msg->position.x;
        monica_current_y_ = msg->position.y;
        
        // Calculate distance traveled in x and y directions
        double dx = monica_current_x_ - monica_start_x_;
        double dy = monica_current_y_ - monica_start_y_;
        monica_distance_traveled_ = std::abs(dx); // Use absolute value for distance
        
        // Print distance traveled in both directions
        RCLCPP_INFO(this->get_logger(), "Monica - X distance: %.3f m, Y distance: %.3f m", dx, dy);
    }
    
    void ross_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!ross_pose_initialized_) {
            ross_start_x_ = msg->position.x;
            ross_start_y_ = msg->position.y;
            ross_pose_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Ross pose initialized at (%.3f, %.3f)", 
                       ross_start_x_, ross_start_y_);
        }
        
        ross_current_x_ = msg->position.x;
        ross_current_y_ = msg->position.y;
        
        // Calculate distance traveled in x and y directions
        double dx = ross_current_x_ - ross_start_x_;
        double dy = ross_current_y_ - ross_start_y_;
        ross_distance_traveled_ = std::abs(dx); // Use absolute value for distance
        
        // Print distance traveled in both directions
        RCLCPP_INFO(this->get_logger(), "Ross - X distance: %.3f m, Y distance: %.3f m", dx, dy);
    }
    
    void control_loop()
    {
        // Check if both robots have initialized poses
        if (!monica_pose_initialized_ || !ross_pose_initialized_) {
            return;
        }
        
        // Check if either robot has reached the target distance
        bool monica_done = monica_distance_traveled_ >= distance_;
        bool ross_done = ross_distance_traveled_ >= distance_;
        
        if (monica_done || ross_done) {
            // Stop both robots
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.linear.y = 0.0;
            stop_cmd.linear.z = 0.0;
            stop_cmd.angular.x = 0.0;
            stop_cmd.angular.y = 0.0;
            stop_cmd.angular.z = 0.0;
            
            // Send stop commands multiple times to ensure they're received
            for (int i = 0; i < 10; ++i) {
                monica_pub_->publish(stop_cmd);
                ross_pub_->publish(stop_cmd);
            }
            
            RCLCPP_INFO(this->get_logger(), "Target distance reached! Monica: %.3f m, Ross: %.3f m - robots stopped",
                       monica_distance_traveled_, ross_distance_traveled_);
            
            // Shutdown the node
            rclcpp::shutdown();
            return;
        }
        
        // Create velocity commands
        geometry_msgs::msg::Twist monica_cmd;
        geometry_msgs::msg::Twist ross_cmd;
        
        // Monica drives in negative x direction (backwards) - only if not done
        if (!monica_done) {
            monica_cmd.linear.x = -speed_;
            monica_cmd.linear.y = 0.0;
            monica_cmd.linear.z = 0.0;
            monica_cmd.angular.x = 0.0;
            monica_cmd.angular.y = 0.0;
            monica_cmd.angular.z = 0.0;
        }
        
        // Ross drives in positive x direction (forwards) - only if not done
        if (!ross_done) {
            ross_cmd.linear.x = speed_;
            ross_cmd.linear.y = 0.0;
            ross_cmd.linear.z = 0.0;
            ross_cmd.angular.x = 0.0;
            ross_cmd.angular.y = 0.0;
            ross_cmd.angular.z = 0.0;
        }
        
        // Publish commands
        monica_pub_->publish(monica_cmd);
        ross_pub_->publish(ross_cmd);
        
        // Log progress occasionally
        static int counter = 0;
        if (counter % 50 == 0) { // Log every second (50 Hz / 50 = 1 Hz)
            RCLCPP_INFO(this->get_logger(), "Progress - Monica: %.3f/%.3f m, Ross: %.3f/%.3f m",
                       monica_distance_traveled_, distance_, ross_distance_traveled_, distance_);
        }
        counter++;
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr monica_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ross_pose_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameters
    double distance_;
    double speed_;
    
    // Monica state
    double monica_start_x_, monica_start_y_;
    double monica_current_x_, monica_current_y_;
    double monica_distance_traveled_;
    bool monica_pose_initialized_;
    
    // Ross state
    double ross_start_x_, ross_start_y_;
    double ross_current_x_, ross_current_y_;
    double ross_distance_traveled_;
    bool ross_pose_initialized_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualLinearConstantVelocityDistanceBasedVicom>();
    
    // Keep the node alive until it shuts itself down
    rclcpp::spin(node);
    return 0;
}