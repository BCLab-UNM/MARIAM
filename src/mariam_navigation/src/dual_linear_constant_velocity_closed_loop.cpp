#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class DualLinearConstantVelocityClosedLoop : public rclcpp::Node
{
public:
    DualLinearConstantVelocityClosedLoop() : Node("dual_linear_constant_velocity_closed_loop")
    {
        // Declare parameters with defaults
        this->declare_parameter("distance", 2.0);
        this->declare_parameter("speed", 0.1);
        this->declare_parameter("kp_angular", 1.5);  // Reduced from 1.0
        this->declare_parameter("kd_angular", 0.3);  // Derivative term to reduce overshoot
        this->declare_parameter("ki_angular", 0.1);  // Integral term for steady-state error
        this->declare_parameter("y_deadband", 0.005); // 5mm deadband to prevent oscillation
        
        // Get parameters
        distance_ = this->get_parameter("distance").as_double();
        speed_ = this->get_parameter("speed").as_double();
        kp_angular_ = this->get_parameter("kp_angular").as_double();
        kd_angular_ = this->get_parameter("kd_angular").as_double();
        ki_angular_ = this->get_parameter("ki_angular").as_double();
        y_deadband_ = this->get_parameter("y_deadband").as_double();
        
        // Initialize state variables
        monica_start_x_ = 0.0;
        monica_start_y_ = 0.0;
        monica_current_x_ = 0.0;
        monica_current_y_ = 0.0;
        monica_distance_traveled_ = 0.0;
        monica_odom_initialized_ = false;
        monica_prev_y_error_ = 0.0;
        monica_y_error_integral_ = 0.0;
        
        ross_start_x_ = 0.0;
        ross_start_y_ = 0.0;
        ross_current_x_ = 0.0;
        ross_current_y_ = 0.0;
        ross_distance_traveled_ = 0.0;
        ross_odom_initialized_ = false;
        ross_prev_y_error_ = 0.0;
        ross_y_error_integral_ = 0.0;
        
        // Distance tolerance
        distance_tolerance_ = 0.01;
        
        // Create publishers
        monica_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("monica/cmd_vel", 10);
        ross_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("ross/cmd_vel", 10);
        
        // Create subscribers
        monica_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "monica/wheel/odom", 10,
            std::bind(&DualLinearConstantVelocityClosedLoop::monica_odom_callback, this, std::placeholders::_1));
        
        ross_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ross/wheel/odom", 10,
            std::bind(&DualLinearConstantVelocityClosedLoop::ross_odom_callback, this, std::placeholders::_1));
        
        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz control loop
            std::bind(&DualLinearConstantVelocityClosedLoop::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Starting dual robot closed-loop drive - Distance: %.2f m, Speed: %.2f m/s, PID: %.2f/%.2f/%.2f",
                   distance_, speed_, kp_angular_, ki_angular_, kd_angular_);
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
    
    void control_loop()
    {
        // Check if both robots have initialized odometry
        if (!monica_odom_initialized_ || !ross_odom_initialized_) {
            return;
        }
        
        // Check if both robots have reached their target distance
        bool monica_done = monica_distance_traveled_ >= (distance_ - distance_tolerance_);
        bool ross_done = ross_distance_traveled_ >= (distance_ - distance_tolerance_);
        
        if (monica_done && ross_done) {
            // Stop both robots
            geometry_msgs::msg::Twist stop_cmd;
            monica_pub_->publish(stop_cmd);
            ross_pub_->publish(stop_cmd);
            
            RCLCPP_INFO(this->get_logger(), "Both robots reached target distance. Monica: %.3f m, Ross: %.3f m",
                       monica_distance_traveled_, ross_distance_traveled_);
            
            // Shutdown the node
            rclcpp::shutdown();
            return;
        }
        
        // Create velocity commands
        geometry_msgs::msg::Twist monica_cmd;
        geometry_msgs::msg::Twist ross_cmd;
        
        // CLOSED-LOOP CONTROL IMPLEMENTATION FOR DIFFERENTIAL DRIVE:
        // This is a HYBRID control system combining:
        // 1. OPEN-LOOP control for x-axis (constant velocity, no feedback)
        // 2. CLOSED-LOOP PROPORTIONAL control for heading (angular velocity with odometry feedback)
        // 3. CLOSED-LOOP BANG-BANG control for distance (binary on/off based on distance threshold)
        
        // Monica control (moves backwards)
        if (!monica_done) {
            // Calculate heading error: use y-displacement to determine required heading correction
            double monica_y_error = monica_current_y_ - monica_start_y_;
            
            // Apply deadband to prevent small oscillations
            if (std::abs(monica_y_error) < y_deadband_) {
                monica_y_error = 0.0;
                monica_y_error_integral_ = 0.0; // Reset integral when in deadband
            } else {
                // Update integral term (with windup protection)
                monica_y_error_integral_ += monica_y_error * 0.02; // dt = 20ms
                // Clamp integral to prevent windup
                monica_y_error_integral_ = std::max(-0.5, std::min(0.5, monica_y_error_integral_));
            }
            
            // Calculate derivative term
            double monica_y_error_derivative = (monica_y_error - monica_prev_y_error_) / 0.02; // dt = 20ms
            monica_prev_y_error_ = monica_y_error;
            
            // PID control: Since Monica faces backwards, invert the correction
            double monica_angular_correction = kp_angular_ * monica_y_error + 
                                             ki_angular_ * monica_y_error_integral_ + 
                                             kd_angular_ * monica_y_error_derivative;
            
            monica_cmd.linear.x = -speed_;  // Open-loop constant velocity (backwards)
            monica_cmd.linear.y = 0.0;      // No lateral movement possible in differential drive
            monica_cmd.angular.z = monica_angular_correction;  // PID angular correction
        }
        
        // Ross control (moves forwards)
        if (!ross_done) {
            // Calculate heading error: use y-displacement to determine required heading correction
            double ross_y_error = ross_current_y_ - ross_start_y_;
            
            // Apply deadband to prevent small oscillations
            if (std::abs(ross_y_error) < y_deadband_) {
                ross_y_error = 0.0;
                ross_y_error_integral_ = 0.0; // Reset integral when in deadband
            } else {
                // Update integral term (with windup protection)
                ross_y_error_integral_ += ross_y_error * 0.02; // dt = 20ms
                // Clamp integral to prevent windup
                ross_y_error_integral_ = std::max(-0.5, std::min(0.5, ross_y_error_integral_));
            }
            
            // Calculate derivative term
            double ross_y_error_derivative = (ross_y_error - ross_prev_y_error_) / 0.02; // dt = 20ms
            ross_prev_y_error_ = ross_y_error;
            
            // PID control: For forward-facing robot, use negative feedback
            double ross_angular_correction = -kp_angular_ * ross_y_error - 
                                           ki_angular_ * ross_y_error_integral_ - 
                                           kd_angular_ * ross_y_error_derivative;
            
            ross_cmd.linear.x = speed_;   // Open-loop constant velocity (forwards)
            ross_cmd.linear.y = 0.0;      // No lateral movement possible in differential drive
            ross_cmd.angular.z = ross_angular_correction;   // PID angular correction
        }
        
        // Publish commands
        monica_pub_->publish(monica_cmd);
        ross_pub_->publish(ross_cmd);
        
        // Log progress occasionally
        static int counter = 0;
        if (counter % 50 == 0) { // Log every second (50 Hz / 50 = 1 Hz)
            RCLCPP_INFO(this->get_logger(), "Progress - Monica: %.3f/%.3f m (y_err: %.3f), Ross: %.3f/%.3f m (y_err: %.3f)",
                       monica_distance_traveled_, distance_, monica_current_y_ - monica_start_y_,
                       ross_distance_traveled_, distance_, ross_current_y_ - ross_start_y_);
        }
        counter++;
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr monica_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ross_odom_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameters
    double distance_;
    double speed_;
    double distance_tolerance_;
    double kp_angular_;
    double ki_angular_;
    double kd_angular_;
    double y_deadband_;
    
    // Monica state
    double monica_start_x_, monica_start_y_;
    double monica_current_x_, monica_current_y_;
    double monica_distance_traveled_;
    bool monica_odom_initialized_;
    double monica_prev_y_error_;
    double monica_y_error_integral_;
    
    // Ross state
    double ross_start_x_, ross_start_y_;
    double ross_current_x_, ross_current_y_;
    double ross_distance_traveled_;
    bool ross_odom_initialized_;
    double ross_prev_y_error_;
    double ross_y_error_integral_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualLinearConstantVelocityClosedLoop>();
    
    // Keep the node alive until it shuts itself down
    rclcpp::spin(node);
    return 0;
}