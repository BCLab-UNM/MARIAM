#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "pid_controller.hpp"

class CoopTrajPID : public rclcpp::Node
{
public:
  CoopTrajPID() : Node("coop_traj_pid") {
    // change QoS settings
    auto qos_profile = rclcpp::QoS(10);
    qos_profile.best_effort();
    qos_profile.durability_volatile();

    ross_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ross/cmd_vel", qos_profile);
    
      monica_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/monica/cmd_vel", qos_profile);

    base1_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/base1_pose",
      qos_profile,
      std::bind(&CoopTrajPID::control_loop, this, std::placeholders::_1)
    );
    
    base2_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/base2_pose",
      qos_profile,
      std::bind(&CoopTrajPID::control_loop, this, std::placeholders::_1)
    );

    // Create base 1 PID controllers
    // Declare PID parameters for three controllers
    this->declare_parameter("base1_pid_x_kp", 1.0);
    this->declare_parameter("base1_pid_x_ki", 0.0);
    this->declare_parameter("base1_pid_x_kd", 0.0);

    this->declare_parameter("base1_pid_y_kp", 1.0);
    this->declare_parameter("base1_pid_y_ki", 0.0);
    this->declare_parameter("base1_pid_y_kd", 0.0);

    this->declare_parameter("base1_pid_theta_kp", 1.0);
    this->declare_parameter("base1_pid_theta_ki", 0.0);
    this->declare_parameter("base1_pid_theta_kd", 0.0);

    double base1_kp_x = this->get_parameter("base1_pid_x_kp").as_double();
    double base1_ki_x = this->get_parameter("base1_pid_x_ki").as_double();
    double base1_kd_x = this->get_parameter("base1_pid_x_kd").as_double();

    double base1_kp_y = this->get_parameter("base1_pid_y_kp").as_double();
    double base1_ki_y = this->get_parameter("base1_pid_y_ki").as_double();
    double base1_kd_y = this->get_parameter("base1_pid_y_kd").as_double();

    double base1_kp_theta = this->get_parameter("base1_pid_theta_kp").as_double();
    double base1_ki_theta = this->get_parameter("base1_pid_theta_ki").as_double();
    double base1_kd_theta = this->get_parameter("base1_pid_theta_kd").as_double();

    base1_pid_x_ = std::make_unique<PIDController>(
      base1_kp_x, base1_ki_x, base1_kd_x);
    base1_pid_y_ = std::make_unique<PIDController>(
      base1_kp_y, base1_ki_y, base1_kd_y);
    base1_pid_theta_ = std::make_unique<PIDController>(
      base1_kp_theta, base1_ki_theta, base1_kd_theta);

    // Create base 2 PID controllers
    // Declare PID parameters for three controllers
    this->declare_parameter("base2_pid_x_kp", 1.0);
    this->declare_parameter("base2_pid_x_ki", 0.0);
    this->declare_parameter("base2_pid_x_kd", 0.0);

    this->declare_parameter("base2_pid_y_kp", 1.0);
    this->declare_parameter("base2_pid_y_ki", 0.0);
    this->declare_parameter("base2_pid_y_kd", 0.0);

    this->declare_parameter("base2_pid_theta_kp", 1.0);
    this->declare_parameter("base2_pid_theta_ki", 0.0);
    this->declare_parameter("base2_pid_theta_kd", 0.0);

    double base2_kp_x = this->get_parameter("base2_pid_x_kp").as_double();
    double base2_ki_x = this->get_parameter("base2_pid_x_ki").as_double();
    double base2_kd_x = this->get_parameter("base2_pid_x_kd").as_double();

    double base2_kp_y = this->get_parameter("base2_pid_y_kp").as_double();
    double base2_ki_y = this->get_parameter("base2_pid_y_ki").as_double();
    double base2_kd_y = this->get_parameter("base2_pid_y_kd").as_double();

    double base2_kp_theta = this->get_parameter("base2_pid_theta_kp").as_double();
    double base2_ki_theta = this->get_parameter("base2_pid_theta_ki").as_double();
    double base2_kd_theta = this->get_parameter("base2_pid_theta_kd").as_double();

    base2_pid_x_ = std::make_unique<PIDController>(
      base2_kp_x, base2_ki_x, base2_kd_x);
    base2_pid_y_ = std::make_unique<PIDController>(
      base2_kp_y, base2_ki_y, base2_kd_y);
    base2_pid_theta_ = std::make_unique<PIDController>(
      base2_kp_theta, base2_ki_theta, base2_kd_theta);

  }

private:
  void control_loop(const geometry_msgs::msg::Pose::SharedPtr msg) {}

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base1_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base2_sub_;

  // PID controllers for base 1
  std::unique_ptr<PIDController> base1_pid_x_;
  std::unique_ptr<PIDController> base1_pid_y_;
  std::unique_ptr<PIDController> base1_pid_theta_;

  // PID controllers for base 2
  std::unique_ptr<PIDController> base2_pid_x_;
  std::unique_ptr<PIDController> base2_pid_y_;
  std::unique_ptr<PIDController> base2_pid_theta_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoopTrajPID>());
  rclcpp::shutdown();
  return 0;
}
