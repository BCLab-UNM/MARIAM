#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>

class DualLinearSinusoidalVelocityOpenLoop : public rclcpp::Node
{
public:
    DualLinearSinusoidalVelocityOpenLoop() : Node("dual_linear_sinusoidal_velocity_open_loop")
    {
        // Declare parameters with defaults
        this->declare_parameter("distance", 2.0);
        this->declare_parameter("max_speed", 0.3);
        this->declare_parameter("min_speed", 0.05);
        this->declare_parameter("wavelength", 4.0);
        
        // Get parameters
        distance_ = this->get_parameter("distance").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        wavelength_ = this->get_parameter("wavelength").as_double();
        
        // Initialize odometry tracking variables
        monica_odom_initialized_ = false;
        ross_odom_initialized_ = false;
        monica_distance_traveled_ = 0.0;
        ross_distance_traveled_ = 0.0;
        driving_started_ = false;

        // Create publishers for both robots
        monica_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("monica/cmd_vel", 10);
        ross_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("ross/cmd_vel", 10);
        
        // Create subscribers for odometry
        monica_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "monica/wheel/odom", 10,
            std::bind(&DualLinearSinusoidalVelocityOpenLoop::monica_odom_callback, this, std::placeholders::_1));
        
        ross_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ross/wheel/odom", 10,
            std::bind(&DualLinearSinusoidalVelocityOpenLoop::ross_odom_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Starting dual robot sinusoidal drive - Distance: %.2f m, Max Speed: %.2f m/s, Min Speed: %.2f m/s, Wavelength: %.2f s",
                    distance_, max_speed_, min_speed_, wavelength_);
        
        // Use a timer to start driving after the node is fully initialized
        start_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                start_timer_->cancel();
                initialize_driving();
            });
    }

private:
    void monica_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!monica_odom_initialized_) {
            monica_start_x_ = msg->pose.pose.position.x;
            monica_start_y_ = msg->pose.pose.position.y;
            monica_odom_initialized_ = true;
        }
        monica_current_x_ = msg->pose.pose.position.x;
        monica_current_y_ = msg->pose.pose.position.y;
        
        // Calculate distance traveled in x direction only
        double dx = monica_current_x_ - monica_start_x_;
        monica_distance_traveled_ = std::abs(dx); // Use absolute value for distance
    }
    
    void ross_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!ross_odom_initialized_) {
            ross_start_x_ = msg->pose.pose.position.x;
            ross_start_y_ = msg->pose.pose.position.y;
            ross_odom_initialized_ = true;
        }
        ross_current_x_ = msg->pose.pose.position.x;
        ross_current_y_ = msg->pose.pose.position.y;
        
        // Calculate distance traveled in x direction only
        double dx = ross_current_x_ - ross_start_x_;
        ross_distance_traveled_ = std::abs(dx); // Use absolute value for distance
    }
    
    void initialize_driving()
    {
        // Check if odometry is initialized
        if (!monica_odom_initialized_ || !ross_odom_initialized_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for odometry initialization...");
            // Reschedule this function to run again
            start_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() {
                    start_timer_->cancel();
                    initialize_driving();
                });
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Odometry initialized. Starting sinusoidal drive...");
        
        // Record start time
        start_time_ = std::chrono::steady_clock::now();
        driving_started_ = true;
        
        // Start control timer at 50 Hz
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz
            std::bind(&DualLinearSinusoidalVelocityOpenLoop::control_callback, this));
    }
    
    void control_callback()
    {
        // Check if either robot has reached target distance
        bool monica_done = monica_distance_traveled_ >= distance_;
        bool ross_done = ross_distance_traveled_ >= distance_;
        
        // Add info message to monitor progress
        RCLCPP_INFO(this->get_logger(), "Monica: %.3f m (done: %s) | Ross: %.3f m (done: %s)", 
                    monica_distance_traveled_, monica_done ? "true" : "false",
                    ross_distance_traveled_, ross_done ? "true" : "false");
        
        if (monica_done || ross_done) {
            control_timer_->cancel();
            stop_robots();
            return;
        }
        
        // Calculate elapsed time and sinusoidal velocity
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time_).count();
        
        // Calculate sinusoidal velocity
        double frequency = 1.0 / wavelength_;  // Convert wavelength to frequency
        double sin_value = std::sin(2.0 * M_PI * frequency * elapsed_time);
        double speed = min_speed_ + (max_speed_ - min_speed_) * (sin_value + 1.0) / 2.0;
        
        // Create velocity messages
        geometry_msgs::msg::Twist monica_cmd;
        geometry_msgs::msg::Twist ross_cmd;
        
        // Monica drives in negative x direction (backwards)
        monica_cmd.linear.x = -speed;
        monica_cmd.linear.y = 0.0;
        monica_cmd.linear.z = 0.0;
        monica_cmd.angular.x = 0.0;
        monica_cmd.angular.y = 0.0;
        monica_cmd.angular.z = 0.0;
        
        // Ross drives in positive x direction (forwards)
        ross_cmd.linear.x = speed;
        ross_cmd.linear.y = 0.0;
        ross_cmd.linear.z = 0.0;
        ross_cmd.angular.x = 0.0;
        ross_cmd.angular.y = 0.0;
        ross_cmd.angular.z = 0.0;
        
        // Publish commands
        monica_pub_->publish(monica_cmd);
        ross_pub_->publish(ross_cmd);
    }
    
    void stop_robots()
    {
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
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        RCLCPP_INFO(this->get_logger(), "Drive complete - Monica traveled: %.2f m, Ross traveled: %.2f m",
                    monica_distance_traveled_, ross_distance_traveled_);
        
        // Shutdown the node
        rclcpp::shutdown();
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr monica_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ross_odom_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Timing
    std::chrono::steady_clock::time_point start_time_;
    
    // Parameters
    double distance_;
    double max_speed_;
    double min_speed_;
    double wavelength_;
    
    // State
    bool driving_started_;
    
    // Odometry tracking
    bool monica_odom_initialized_;
    bool ross_odom_initialized_;
    double monica_start_x_, monica_start_y_;
    double ross_start_x_, ross_start_y_;
    double monica_current_x_, monica_current_y_;
    double ross_current_x_, ross_current_y_;
    double monica_distance_traveled_;
    double ross_distance_traveled_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualLinearSinusoidalVelocityOpenLoop>();
    
    // Keep the node alive until it shuts itself down
    rclcpp::spin(node);
    return 0;
}