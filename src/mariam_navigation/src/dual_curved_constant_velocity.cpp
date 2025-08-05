#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

enum class RobotState {
    STRAIGHT_MOTION,
    TURNING
};

class DualCurvedMotionController : public rclcpp::Node
{
private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ross_odom_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameters
    double linear_speed_;
    double turn_radius_;
    double initial_separation_;
    double angular_speed_;
    
    // Robot states
    RobotState ross_state_;
    
    // Ross state variables (only Ross needs distance tracking)
    double ross_start_x_;
    double ross_current_x_;
    double ross_distance_traveled_;
    bool ross_odom_initialized_;

public:
    DualCurvedMotionController() : Node("dual_curved_motion_controller")
    {
        // Declare parameters with defaults
        this->declare_parameter("linear_speed", 0.1);
        this->declare_parameter("turn_radius", 1.0);
        this->declare_parameter("initial_separation", 0.86);
        
        // Get parameters
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        turn_radius_ = this->get_parameter("turn_radius").as_double();
        initial_separation_ = this->get_parameter("initial_separation").as_double();
        
        // Calculate derived parameters
        angular_speed_ = linear_speed_ / turn_radius_; // v = ω * r
        
        // Initialize state variables
        ross_state_ = RobotState::STRAIGHT_MOTION; // Ross starts with straight motion
        
        // Initialize Ross odometry tracking
        ross_start_x_ = 0.0;
        ross_current_x_ = 0.0;
        ross_distance_traveled_ = 0.0;
        ross_odom_initialized_ = false;
        
        // Create publishers
        monica_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("monica/cmd_vel", 10);
        ross_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("ross/cmd_vel", 10);
        
        // Create subscriber (only need Ross odom for distance tracking)
        ross_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ross/wheel/odom", 10,
            std::bind(&DualCurvedMotionController::ross_odom_callback, this, std::placeholders::_1));
        
        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50 Hz control loop
            std::bind(&DualCurvedMotionController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Starting dual robot curved motion controller");
        RCLCPP_INFO(this->get_logger(), "Linear speed: %.3f m/s, Turn radius: %.3f m, Angular speed: %.3f rad/s", 
                   linear_speed_, turn_radius_, angular_speed_);
        RCLCPP_INFO(this->get_logger(), "Monica starts turning immediately, Ross will turn after %.3f m", 
                   initial_separation_);
    }
    
    void ross_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!ross_odom_initialized_) {
            ross_start_x_ = msg->pose.pose.position.x;
            ross_odom_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Ross odometry initialized at x = %.3f", ross_start_x_);
        }
        
        ross_current_x_ = msg->pose.pose.position.x;
        
        // Calculate distance traveled in x direction (only during straight motion)
        if (ross_state_ == RobotState::STRAIGHT_MOTION) {
            double dx = ross_current_x_ - ross_start_x_;
            ross_distance_traveled_ = std::abs(dx);
        }
    }
    
    void control_loop()
    {
        // Check if Ross odometry is initialized
        if (!ross_odom_initialized_) {
            return;
        }
        
        // Handle Ross's state transition
        if (ross_state_ == RobotState::STRAIGHT_MOTION && ross_distance_traveled_ >= initial_separation_) {
            ross_state_ = RobotState::TURNING;
            RCLCPP_INFO(this->get_logger(), "Ross completed %.3f m straight motion, now turning forever", 
                       ross_distance_traveled_);
        }
        
        // Create velocity commands
        geometry_msgs::msg::Twist monica_cmd;
        geometry_msgs::msg::Twist ross_cmd;
        
        // Monica is always turning (starts immediately)
        monica_cmd.linear.x = -linear_speed_; // Negative because Monica faces backwards
        monica_cmd.angular.z = angular_speed_; // Left turn
        
        // Ross commands depend on state
        if (ross_state_ == RobotState::STRAIGHT_MOTION) {
            ross_cmd.linear.x = linear_speed_;
            ross_cmd.angular.z = 0.0;
        } else { // TURNING
            ross_cmd.linear.x = linear_speed_;
            ross_cmd.angular.z = angular_speed_; // Left turn
        }
        
        // Publish commands
        monica_pub_->publish(monica_cmd);
        ross_pub_->publish(ross_cmd);
        
        // Log progress occasionally
        static int counter = 0;
        if (counter % 100 == 0) { // Log every 2 seconds (50 Hz / 100 = 0.5 Hz)
            if (ross_state_ == RobotState::STRAIGHT_MOTION) {
                RCLCPP_INFO(this->get_logger(), "Monica: TURNING, Ross: STRAIGHT (%.3f/%.3f m)",
                           ross_distance_traveled_, initial_separation_);
            } else {
                RCLCPP_INFO(this->get_logger(), "Monica: TURNING, Ross: TURNING");
            }
        }
        counter++;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualCurvedMotionController>();
    
    // Keep the node alive
    rclcpp::spin(node);
    return 0;
}