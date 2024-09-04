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
        "virtual_pose",
        10,
        std::bind(&AdmittanceController::callback, this, _1)
      );

      force_sub = this->create_subscription<Float64>(
        "force_reading",
        10,
        std::bind(&AdmittanceController::force_callback, this, _1)
      );
    }

  private:
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
    rclcpp::Subscription<Pose>::SharedPtr virtual_pose_sub;
    rclcpp::Subscription<Float64>::SharedPtr force_sub;
    double most_recent_force_reading = 0;
    const int STIFFNESS = 25;

    void callback(const Pose::SharedPtr msg) {
      auto new_msg = Pose();
      new_msg.position = msg->position;
      new_msg.orientation = msg->orientation;
      new_msg.position.y -= most_recent_force_reading / STIFFNESS;
      this->pose_publisher->publish(new_msg);
    }

    void force_callback(const Float64::SharedPtr msg) {
      most_recent_force_reading = msg->data;
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdmittanceController>());
  rclcpp::shutdown();
  return 0;
}