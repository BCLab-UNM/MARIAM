// C++
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace geometry_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;


class AdmittanceController : public rclcpp::Node {
  public:
    AdmittanceController() : Node("admittance_controller") {
      // =============== creating publishers and subscriptions ===============
      pose_publisher = this->create_publisher<Pose>(
        "px100_target_pose",
        10
      );

      virtual_pose_sub = this->create_subscription<Pose>(
        "px100_virtual_pose",
        10,
        std::bind(&AdmittanceController::admittance_control_callback, this, _1)
      );

      force_sub = this->create_subscription<Float32>(
        "force",
        10,
        std::bind(&AdmittanceController::force_callback, this, _1)
      );

      mass_sub = this->create_subscription<Float64>(
        "admittance_control/mass",
        10,
        std::bind(&AdmittanceController::mass_callback, this, _1)
      );
      
      damping_sub = this->create_subscription<Float64>(
        "admittance_control/damping",
        10,
        std::bind(&AdmittanceController::damping_callback, this, _1)
      );
      
      stiffness_sub = this->create_subscription<Float64>(
        "admittance_control/stiffness",
        10,
        std::bind(&AdmittanceController::stiffness_callback, this, _1)
      );

      // =============== creating parameters ===============
      // declaring parameters for the initial values of
      // mass, damping, and stiffness
      this->declare_parameter("mass",       5.0);
      this->declare_parameter("damping",   10.0);
      this->declare_parameter("stiffness", 15.0);

      // storing the initial values into variables
      this->get_parameter("mass",      mass);
      this->get_parameter("damping",   damping);
      this->get_parameter("stiffness", stiffness);
    }

  private:
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
    rclcpp::Subscription<Pose>::SharedPtr virtual_pose_sub;
    rclcpp::Subscription<Float32>::SharedPtr force_sub;
    rclcpp::Subscription<Float64>::SharedPtr mass_sub;
    rclcpp::Subscription<Float64>::SharedPtr damping_sub;
    rclcpp::Subscription<Float64>::SharedPtr stiffness_sub;
    
    // most recent force reading
    double force_reading = 0;
    
    // previous desired position
    double position = 0;
    
    // previous desired velocity
    double velocity = 0;
    double step_size = 0.002;

    double mass;
    double damping;
    double stiffness;
    
    const rclcpp::Logger LOGGER = rclcpp::get_logger("admittance_controller");

    void admittance_control_callback(const Pose::SharedPtr msg) {
      auto new_msg = Pose();
      new_msg.position = msg->position;
      new_msg.orientation = msg->orientation;
      // Using Hooke's law to compute desired position
      // new_msg.position.y -= force_reading / stiffness;
      
      // Using Euler's method to compute desired position
      // acceleration is in meters per second squared
      double acceleration = (force_reading-damping*velocity-stiffness*position)/mass;
      // velocity is in meters per second
      velocity += step_size*acceleration;
      // position is in meters
      position += step_size*velocity;
      new_msg.position.y -= position;
      
      this->pose_publisher->publish(new_msg);
    }

    void force_callback(const Float32::SharedPtr msg) {
      force_reading = msg->data;
    }

    void mass_callback(const Float64::SharedPtr msg) {
      RCLCPP_INFO(LOGGER, "Updating mass from %.2f to %.2f", mass, msg->data);
      mass = msg->data;
    }
    
    void damping_callback(const Float64::SharedPtr msg) {
      RCLCPP_INFO(LOGGER, "Updating damping from %.2f to %.2f", damping, msg->data);
      damping = msg->data;
    }
    
    void stiffness_callback(const Float64::SharedPtr msg) {
      RCLCPP_INFO(LOGGER, "Updating stiffness from %.2f to %.2f", stiffness, msg->data);
      stiffness = msg->data;
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdmittanceController>());
  rclcpp::shutdown();
  return 0;
}