#include <chrono>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;

class VirtualPosePublisher : public rclcpp::Node {
  public:
    VirtualPosePublisher() : Node("virtual_pose_publisher") {
      pose_publisher = this->create_publisher<Pose>(
        "px100_virtual_pose",
        10
      );
      // position parameters
      this->declare_parameter("x_pos", 0.0);
      this->declare_parameter("y_pos", 0.250048);
      this->declare_parameter("z_pos", 0.098);
      
      // quaternion parameters
      this->declare_parameter("w", 0.707);
      this->declare_parameter("x", 0.0);
      this->declare_parameter("y", 0.0);
      this->declare_parameter("z", 0.707);

      this->declare_parameter("frequency", 1.0);


      this->get_parameter("x_pos", x_pos);
      this->get_parameter("y_pos", y_pos);
      this->get_parameter("z_pos", z_pos_param);
      
      this->get_parameter("w", w);
      this->get_parameter("x", x);
      this->get_parameter("y", y);
      this->get_parameter("z", z);
      
      this->get_parameter("frequency", frequency);

      z_pos = z_pos_param;

      param_callback_handle = this->add_on_set_parameters_callback(
        std::bind(&VirtualPosePublisher::parametersCallback, this, std::placeholders::_1));

      timer = this->create_wall_timer(
        frequency * 1s,
        std::bind(&VirtualPosePublisher::callback, this)
      );
    }

    private:
      double x_pos;
      double y_pos;
      double z_pos_param;
      double z_pos;
      double w;
      double x;
      double y;
      double z;
      double frequency;
      rclcpp::TimerBase::SharedPtr timer;
      rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
      OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
      const rclcpp::Logger LOGGER = rclcpp::get_logger("virtual_pose_publisher");

      void callback() {
        auto msg = Pose();

        if (z_pos != z_pos_param) {
          double sign = (std::signbit(z_pos_param - z_pos) == 0) ? 1 : -1;
          // RCLCPP_INFO(LOGGER, "sign: %.4f", sign);
          z_pos += sign * 0.0001;
          // RCLCPP_INFO(LOGGER, "updated z_pos to: %.4f", z_pos);
        }

        msg.position.x = x_pos;
        msg.position.y = y_pos;
        msg.position.z = z_pos;

        msg.orientation.w = w;
        msg.orientation.x = x;
        msg.orientation.y = y;
        msg.orientation.z = z;
        
        // RCLCPP_INFO(LOGGER,
        //   "Publishing: (%f, %f, %f)\n(%f, %f, %f, %f)",
        //   x_pos,
        //   y_pos,
        //   z_pos,
        //   w,
        //   x,
        //   y,
        //   z
        // );

        pose_publisher->publish(msg);
      }

      rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters
      ) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        const auto param = parameters[0];
        
        if (param.get_name() == "z_pos") {
          z_pos_param = param.as_double();
          RCLCPP_INFO(LOGGER, "Setting z_pos_param to: %.4f", z_pos_param);
        }
    
        return result;
      }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualPosePublisher>());
  rclcpp::shutdown();
  return 0;
}