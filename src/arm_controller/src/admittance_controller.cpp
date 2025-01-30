// C++
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace geometry_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;


class AdmittanceController : public rclcpp::Node {
  public:
    AdmittanceController() : Node("admittance_controller") {
      pose_publisher = this->create_publisher<Pose>(
        "high_freq_target_pose",
        10
      );

      virtual_pose_sub = this->create_subscription<Pose>(
        "high_freq_virtual_pose",
        10,
        std::bind(&AdmittanceController::callback, this, _1)
      );

      force_sub = this->create_subscription<Float64>(
        "force",
        10,
        std::bind(&AdmittanceController::force_callback, this, _1)
      );
    }

  private:
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
    rclcpp::Subscription<Pose>::SharedPtr virtual_pose_sub;
    rclcpp::Subscription<Float64>::SharedPtr force_sub;
    double force_reading = 0;
    double position = 0;
    double velocity = 0;
    double step_size = 0.002;
    const double MASS = 5;
    const double DAMPING = 10;
    const double STIFFNESS = 15;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("admittance_controller");

    void callback(const Pose::SharedPtr msg) {
      auto new_msg = Pose();
      new_msg.position = msg->position;
      new_msg.orientation = msg->orientation;
      // Using Hooke's law to compute desired position
      // new_msg.position.y -= force_reading / STIFFNESS;
      
      // Using Euler's method to compute desired position
      double acceleration = (force_reading-DAMPING*velocity-STIFFNESS*position)/MASS;
      velocity += step_size*acceleration;
      position += step_size*velocity;
      new_msg.position.y -= position;
      
      this->pose_publisher->publish(new_msg);
      RCLCPP_INFO(LOGGER,
        "Position:\n{%.4f,%.4f,%.4f}\nOrientation:\n{%.4f,%.4f,%.4f,%.4f}",
        new_msg.position.x,
        new_msg.position.y,
        new_msg.position.z,
        new_msg.orientation.w,
        new_msg.orientation.x,
        new_msg.orientation.y,
        new_msg.orientation.z
      );
      RCLCPP_INFO(LOGGER,
        "Accel: %f\nVel: %f\nPos:%f",
        acceleration,
        velocity,
        position
      );
    }

    void force_callback(const Float64::SharedPtr msg) {
      force_reading = msg->data;
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdmittanceController>());
  rclcpp::shutdown();
  return 0;
}