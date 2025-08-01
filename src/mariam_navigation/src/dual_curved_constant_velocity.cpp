#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

enum class RobotState {
    WAITING,
    STRAIGHT_MOTION,
    TURNING,
    FINAL_STRAIGHT,
    COMPLETED
};

class DualCurvedMotionController : public rclcpp::Node
{
public:
    DualCurvedMotionController() : Node("dual_curved_motion_controller")
    {
        // Declare parameters with defaults
        this->declare_parameter("linear_speed", 0.1);
        this->declare_parameter("turn_radius", 1.0);
        this->declare_parameter("initial_separation", 0.85);
        
        // Get parameters
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        turn_radius_ = this->get_parameter("turn_radius").as_double();
        initial_separation_ = this->get_parameter("initial_separation").as_double();
        
        // Calculate derived parameters
        angular_speed_ = linear_speed_ / turn_radius_; // v = ω * r
        turn_arc_length_ = (M_PI / 2.0) * turn_radius_; // 90° = π/2 radians
        
        // Initialize state variables
        monica_state_ = RobotState::TURNING; // Monica starts turning immediately
        ross_state_ = RobotState::STRAIGHT_MOTION; // Ross starts with straight motion
        
        // Initialize odometry tracking
        monica_start_x_ = 0.0;
        monica_start_y_ = 0.0;
        monica_current_x_ = 0.0;
        monica_current_y_ = 0.0;
        monica_distance_traveled_ = 0.0;
        monica_odom_initialized_ = false;
        
        ross_start_x_ = 0.0;
        ross_start_y_ = 0.0;
        ross_current_x_ = 0.0;
        ross_current_y_ = 0.0;
        ross_distance_traveled_ = 0.0;
        ross_odom_initialized_ = false;
        
        // Create publishers
        monica_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("monica/cmd_vel", 10);
        ross_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("ross/cmd_vel", 10);
        
        // Create subscribers
        monica_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "monica/wheel/odom", 10,
            std::bind(&DualCurvedMotionController::monica_odom_callback, this, std::placeholders::_1));
        
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
        RCLCPP_INFO(this->get_logger(), "Turn arc length: %.3f m, Initial separation: %.3f m", 
                   turn_arc_length_, initial_separation_);
    }

private:
    void monica_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!monica_odom_initialized_) {
            monica_start_x_ = msg->pose.pose.position.x;
            monica_start_y_ = msg->pose.pose.position.y;
            monica_odom_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Monica odometry initialized at (%.3f, %.3f)", 
                       monica_start_x_, monica_start_y_);
        }
        
        monica_current_x_ = msg->pose.pose.position.x;
        monica_current_y_ = msg->pose.pose.position.y;
        
        // Calculate distance traveled based on current state
        if (monica_state_ == RobotState::TURNING) {
            // For turning, calculate arc length based on total displacement
            double dx = monica_current_x_ - monica_start_x_;
            double dy = monica_current_y_ - monica_start_y_;
            monica_distance_traveled_ = sqrt(dx*dx + dy*dy);
        } else if (monica_state_ == RobotState::FINAL_STRAIGHT) {
            // For final straight motion, measure from start of this phase
            double dx = monica_current_x_ - monica_final_start_x_;
            double dy = monica_current_y_ - monica_final_start_y_;
            monica_distance_traveled_ = sqrt(dx*dx + dy*dy);
        }
    }
    
    void ross_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!ross_odom_initialized_) {
            ross_start_x_ = msg->pose.pose.position.x;
            ross_start_y_ = msg->pose.pose.position.y;
            ross_odom_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Ross odometry initialized at (%.3f, %.3f)", 
                       ross_start_x_, ross_start_y_);
        }
        
        ross_current_x_ = msg->pose.pose.position.x;
        ross_current_y_ = msg->pose.pose.position.y;
        
        // Calculate distance traveled based on current state
        if (ross_state_ == RobotState::STRAIGHT_MOTION) {
            // For straight motion, measure x-direction distance
            double dx = ross_current_x_ - ross_start_x_;
            ross_distance_traveled_ = std::abs(dx);
        } else if (ross_state_ == RobotState::TURNING) {
            // For turning, calculate arc length from start of turn
            double dx = ross_current_x_ - ross_turn_start_x_;
            double dy = ross_current_y_ - ross_turn_start_y_;
            ross_distance_traveled_ = sqrt(dx*dx + dy*dy);
        }
    }
    
    void control_loop()
    {
        // Check if both robots have initialized odometry
        if (!monica_odom_initialized_ || !ross_odom_initialized_) {
            return;
        }
        
        // Handle Monica's state machine
        handle_monica_state();
        
        // Handle Ross's state machine
        handle_ross_state();
        
        // Create and send velocity commands
        geometry_msgs::msg::Twist monica_cmd;
        geometry_msgs::msg::Twist ross_cmd;
        
        // Monica commands (remember Monica's coordinate frame is flipped)
        if (monica_state_ == RobotState::TURNING) {
            monica_cmd.linear.x = -linear_speed_; // Negative because Monica faces backwards
            monica_cmd.angular.z = -angular_speed_; // Negative for left turn in Monica's frame
        } else if (monica_state_ == RobotState::FINAL_STRAIGHT) {
            monica_cmd.linear.x = -linear_speed_; // Continue backwards motion
            monica_cmd.angular.z = 0.0;
        }
        // COMPLETED or WAITING states have zero velocities (default initialization)
        
        // Ross commands
        if (ross_state_ == RobotState::STRAIGHT_MOTION) {
            ross_cmd.linear.x = linear_speed_;
            ross_cmd.angular.z = 0.0;
        } else if (ross_state_ == RobotState::TURNING) {
            ross_cmd.linear.x = linear_speed_;
            ross_cmd.angular.z = angular_speed_; // Positive for left turn
        }
        // COMPLETED or WAITING states have zero velocities (default initialization)
        
        // Publish commands
        monica_pub_->publish(monica_cmd);
        ross_pub_->publish(ross_cmd);
        
        // Check if both robots are completed
        if (monica_state_ == RobotState::COMPLETED && ross_state_ == RobotState::COMPLETED) {
            RCLCPP_INFO(this->get_logger(), "Both robots completed their curved motion sequence!");
            
            // Send stop commands multiple times to ensure they're received
            geometry_msgs::msg::Twist stop_cmd;
            for (int i = 0; i < 10; ++i) {
                monica_pub_->publish(stop_cmd);
                ross_pub_->publish(stop_cmd);
            }
            
            // Shutdown the node
            rclcpp::shutdown();
            return;
        }
        
        // Log progress occasionally
        static int counter = 0;
        if (counter % 50 == 0) { // Log every second (50 Hz / 50 = 1 Hz)
            RCLCPP_INFO(this->get_logger(), "Monica state: %s (%.3f m), Ross state: %s (%.3f m)",
                       state_to_string(monica_state_).c_str(), monica_distance_traveled_,
                       state_to_string(ross_state_).c_str(), ross_distance_traveled_);
        }
        counter++;
    }
    
    void handle_monica_state()
    {
        switch (monica_state_) {
            case RobotState::TURNING:
                if (monica_distance_traveled_ >= turn_arc_length_) {
                    monica_state_ = RobotState::FINAL_STRAIGHT;
                    // Record starting position for final straight motion
                    monica_final_start_x_ = monica_current_x_;
                    monica_final_start_y_ = monica_current_y_;
                    monica_distance_traveled_ = 0.0; // Reset for next phase
                    RCLCPP_INFO(this->get_logger(), "Monica completed turn, starting final straight motion");
                }
                break;
                
            case RobotState::FINAL_STRAIGHT:
                if (monica_distance_traveled_ >= initial_separation_) {
                    monica_state_ = RobotState::COMPLETED;
                    RCLCPP_INFO(this->get_logger(), "Monica completed final straight motion");
                }
                break;
                
            default:
                break;
        }
    }
    
    void handle_ross_state()
    {
        switch (ross_state_) {
            case RobotState::STRAIGHT_MOTION:
                if (ross_distance_traveled_ >= initial_separation_) {
                    ross_state_ = RobotState::TURNING;
                    // Record starting position for turn
                    ross_turn_start_x_ = ross_current_x_;
                    ross_turn_start_y_ = ross_current_y_;
                    ross_distance_traveled_ = 0.0; // Reset for turn phase
                    RCLCPP_INFO(this->get_logger(), "Ross completed straight motion, starting turn");
                }
                break;
                
            case RobotState::TURNING:
                if (ross_distance_traveled_ >= turn_arc_length_) {
                    ross_state_ = RobotState::COMPLETED;
                    RCLCPP_INFO(this->get_logger(), "Ross completed turn");
                }
                break;
                
            default:
                break;
        }
    }
    
    std::string state_to_string(RobotState state)
    {
        switch (state) {
            case RobotState::WAITING: return "WAITING";
            case RobotState::STRAIGHT_MOTION: return "STRAIGHT";
            case RobotState::TURNING: return "TURNING";
            case RobotState::FINAL_STRAIGHT: return "FINAL_STRAIGHT";
            case RobotState::COMPLETED: return "COMPLETED";
            default: return "UNKNOWN";
        }
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
    double linear_speed_;
    double turn_radius_;
    double initial_separation_;
    double angular_speed_;
    double turn_arc_length_;
    
    // Robot states
    RobotState monica_state_;
    RobotState ross_state_;
    
    // Monica state variables
    double monica_start_x_, monica_start_y_;
    double monica_current_x_, monica_current_y_;
    double monica_final_start_x_, monica_final_start_y_; // For final straight phase
    double monica_distance_traveled_;
    bool monica_odom_initialized_;
    
    // Ross state variables
    double ross_start_x_, ross_start_y_;
    double ross_current_x_, ross_current_y_;
    double ross_turn_start_x_, ross_turn_start_y_; // For turn phase
    double ross_distance_traveled_;
    bool ross_odom_initialized_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualCurvedMotionController>();
    
    // Keep the node alive until it shuts itself down
    rclcpp::spin(node);
    return 0;
}