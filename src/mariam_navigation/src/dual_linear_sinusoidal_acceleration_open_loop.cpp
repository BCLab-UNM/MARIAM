#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>

class DualLinearSinusoidalAccelerationOpenLoop : public rclcpp::Node
{
public:
    DualLinearSinusoidalAccelerationOpenLoop() : Node("dual_linear_sinusoidal_acceleration_open_loop")
    {
        // Declare parameters with defaults
        this->declare_parameter("distance", 2.0);
        this->declare_parameter("max_acceleration", 0.5);
        this->declare_parameter("min_acceleration", 0.1);
        this->declare_parameter("wavelength", 4.0);
        this->declare_parameter("initial_velocity", 0.1);
        this->declare_parameter("max_velocity", 5.0);
        
        // Get parameters
        distance_ = this->get_parameter("distance").as_double();
        max_acceleration_ = this->get_parameter("max_acceleration").as_double();
        min_acceleration_ = this->get_parameter("min_acceleration").as_double();
        wavelength_ = this->get_parameter("wavelength").as_double();
        initial_velocity_ = this->get_parameter("initial_velocity").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        
        // Initialize odometry tracking variables
        monica_odom_initialized_ = false;
        ross_odom_initialized_ = false;
        monica_distance_traveled_ = 0.0;
        ross_distance_traveled_ = 0.0;
        driving_started_ = false;
        
        // Initialize velocity tracking
        monica_velocity_ = initial_velocity_;
        ross_velocity_ = initial_velocity_;

        // Create publishers for both robots
        monica_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("monica/cmd_vel", 10);
        ross_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("ross/cmd_vel", 10);
        
        // Create subscribers for odometry
        monica_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "monica/wheel/odom", 10,
            std::bind(&DualLinearSinusoidalAccelerationOpenLoop::monica_odom_callback, this, std::placeholders::_1));
        
        ross_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ross/wheel/odom", 10,
            std::bind(&DualLinearSinusoidalAccelerationOpenLoop::ross_odom_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Starting dual robot sinusoidal acceleration drive - Distance: %.2f m, Max Accel: %.2f m/s², Min Accel: %.2f m/s², Wavelength: %.2f s, Initial Vel: %.2f m/s",
                    distance_, max_acceleration_, min_acceleration_, wavelength_, initial_velocity_);
        
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
        
        RCLCPP_INFO(this->get_logger(), "Odometry initialized. Starting sinusoidal acceleration drive...");
        
        // Record start time
        start_time_ = std::chrono::steady_clock::now();
        last_time_ = start_time_;
        driving_started_ = true;
        
        // Start control timer at 50 Hz
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz
            std::bind(&DualLinearSinusoidalAccelerationOpenLoop::control_callback, this));
    }
    
    void control_callback()
    {
        // Check if either robot has reached target distance
        bool monica_done = monica_distance_traveled_ >= distance_;
        bool ross_done = ross_distance_traveled_ >= distance_;
        
        // Add info message to monitor progress
        RCLCPP_INFO(this->get_logger(), "Monica: %.3f m (done: %s, vel: %.3f) | Ross: %.3f m (done: %s, vel: %.3f)", 
                    monica_distance_traveled_, monica_done ? "true" : "false", monica_velocity_,
                    ross_distance_traveled_, ross_done ? "true" : "false", ross_velocity_);
        
        if (monica_done || ross_done) {
            control_timer_->cancel();
            stop_robots();
            return;
        }
        
        // Calculate elapsed time and time delta
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time_).count();
        double dt = std::chrono::duration<double>(current_time - last_time_).count();
        last_time_ = current_time;
        
        // Calculate sinusoidal acceleration
        double frequency = 1.0 / wavelength_;  // Convert wavelength to frequency
        double sin_value = std::sin(2.0 * M_PI * frequency * elapsed_time);
        double acceleration = min_acceleration_ + (max_acceleration_ - min_acceleration_) * (sin_value + 1.0) / 2.0;
        
        // Update velocities using acceleration and time delta
        monica_velocity_ += acceleration * dt;
        ross_velocity_ += acceleration * dt;
        
        // Clamp velocities to maximum limit to prevent runaway
        monica_velocity_ = std::min(monica_velocity_, max_velocity_);
        ross_velocity_ = std::min(ross_velocity_, max_velocity_);
        
        // Also prevent negative velocities for safety
        monica_velocity_ = std::max(monica_velocity_, 0.0);
        ross_velocity_ = std::max(ross_velocity_, 0.0);
        
        // Create velocity messages
        geometry_msgs::msg::Twist monica_cmd;
        geometry_msgs::msg::Twist ross_cmd;
        
        // Monica drives in negative x direction (backwards)
        monica_cmd.linear.x = -monica_velocity_;
        monica_cmd.linear.y = 0.0;
        monica_cmd.linear.z = 0.0;
        monica_cmd.angular.x = 0.0;
        monica_cmd.angular.y = 0.0;
        monica_cmd.angular.z = 0.0;
        
        // Ross drives in positive x direction (forwards)
        ross_cmd.linear.x = ross_velocity_;
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
        
        RCLCPP_INFO(this->get_logger(), "Drive complete - Monica traveled: %.2f m (final vel: %.2f m/s), Ross traveled: %.2f m (final vel: %.2f m/s)",
                    monica_distance_traveled_, monica_velocity_, ross_distance_traveled_, ross_velocity_);
        
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
    std::chrono::steady_clock::time_point last_time_;
    
    // Parameters
    double distance_;
    double max_acceleration_;
    double min_acceleration_;
    double wavelength_;
    double initial_velocity_;
    double max_velocity_;
    
    // State
    bool driving_started_;
    double monica_velocity_;
    double ross_velocity_;
    
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
    auto node = std::make_shared<DualLinearSinusoidalAccelerationOpenLoop>();
    
    // Keep the node alive until it shuts itself down
    rclcpp::spin(node);
    return 0;
}