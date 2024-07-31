#include <memory>
#include <functional>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "arm_controller/msg/constrained_pose.hpp"
// for publishing to /commands/joy_processed
#include <interbotix_xs_msgs/msg/arm_joy.hpp>

using std::placeholders::_1;
using namespace arm_controller::msg;
using namespace visualization_msgs::msg;
using namespace geometry_msgs::msg;
using namespace interbotix_xs_msgs::msg;


// Xbox 360 Controller button mappings
static std::map<std::string, int> btn_map = 
{
  // TODO: update these entries to publish poses for lifting
  // {"GRIPPER_PWM_DEC", 0},  // buttons start here
  {"MOVE_FORWARD", 1}, // B
  {"LIFT", 2},       // X
  {"START_LIFT", 3}, // Y
  {"WAIST_CCW", 4},
  {"WAIST_CW", 5},
  {"SLEEP_POSE", 6},
  {"HOME_POSE", 7},
  {"TORQUE_ENABLE", 8},
  {"FLIP_EE_X", 9},
  // {"FLIP_EE_ROLL", 10},
  {"EE_X", 0},             // axes start here
  {"EE_Z", 1},
  // {"EE_Y_INC", 2},
  // {"EE_ROLL", 3},
  {"EE_PITCH", 4},
  // {"EE_Y_DEC", 5},
  {"SPEED_TYPE", 6},
  {"SPEED", 7}
};

class JoyInputHandler : public rclcpp::Node
{
  public:
    JoyInputHandler() : Node("joy_input_handler")
    {
      pose_publisher_ = this->create_publisher<ConstrainedPose>(
        "joy_target_pose",
        1
      );

      goal_pose_joy_publisher_ = this->create_publisher<Marker>(
        "goal_pose_joy",
        10
      );

      pub_joy_cmd = this->create_publisher<interbotix_xs_msgs::msg::ArmJoy>(
        "commands/joy_processed",
        10
      );

      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        std::bind(&JoyInputHandler::joy_callback, this, _1)
      );

      raw_joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "commands/joy_raw",
        10,
        std::bind(&JoyInputHandler::raw_joy_handler, this, _1)
      );

      test_pose1.position.x = 0.250048;
      test_pose1.position.y = 0;
      test_pose1.position.z = 0.098;
      test_pose1.orientation.x =  0.0;
      test_pose1.orientation.y =  0.0;
      test_pose1.orientation.z =  0.0;
      test_pose1.orientation.w =  1.0;

      test_pose2.position.x = 0.250048;
      test_pose2.position.y = 0;
      test_pose2.position.z = 0.244;
      test_pose2.orientation.x =  0.0;
      test_pose2.orientation.y =  0.0;
      test_pose2.orientation.z =  0.0;
      test_pose2.orientation.w =  1.0;

      this->declare_parameter("threshold", 0.75);
      this->get_parameter("threshold", threshold);
    }

  private:
    rclcpp::Publisher<ConstrainedPose>::SharedPtr pose_publisher_;
    rclcpp::Publisher<Marker>::SharedPtr goal_pose_joy_publisher_;
    rclcpp::Publisher<interbotix_xs_msgs::msg::ArmJoy>::SharedPtr pub_joy_cmd;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr raw_joy_subscription_;
    Pose curr_pose;
    Pose test_pose1;
    Pose test_pose2;
    Pose pose_experiment_home;
    interbotix_xs_msgs::msg::ArmJoy prev_joy_cmd;
    float theta = 0;
    bool apply_constraints = false;
    double threshold;
    const rclcpp::Logger LOGGER = rclcpp::get_logger("joy_input_handler");

    /**
     * This method is the callback function for /joy.
     * It reads the message from /joy and performs specific actions
     * depending on which button was pressed.
     * xbox360 layout can be found here: http://wiki.ros.org/joy
     * 
     * Controls:
     * Left stick to move the target pose in the y and z directions.
     * Right stick to move the target pose in the x direction.
     * 'x' to publish the target pose.
     * 'Back' to publish the Sleep pose.
     * 'Start' to publish the Home pose.
     * 'LB' & 'RB' to publish test_pose1 & test_pose2, respectively.
     * 'Y' to toggle path constraints.
     * 
     * @msg: controller input
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      Pose new_pose = curr_pose;

      // LB
      if (msg->buttons[4] == 1) 
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing test_pose1\n\n");
        curr_pose = test_pose1;
        this->publish_marker(test_pose1);
        this->publish_pose(test_pose1, true, "none");
      }

      // RB
      if(msg->buttons[5] == 1)
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing test_pose2\n\n");
        curr_pose = test_pose2;
        this->publish_marker(test_pose2);
        this->publish_pose(new_pose, true, "none");
      }

      // Back
      if(msg->buttons[6] == 1)
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing Sleep pose\n\n");
        this->publish_pose(new_pose, false, "Sleep");
      }

      // Start
      if(msg->buttons[7] == 1)
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing Home pose\n\n");
        this->publish_pose(new_pose, false, "Home");
      }
      
      // X: publish pose
      if (msg->buttons[2])
      {
        RCLCPP_INFO(LOGGER, "\n\nPublishing target pose\n\n");
        this->publish_pose(new_pose, apply_constraints, "none");
      }

      // Update pose
      curr_pose.position.x +=  (0.001)*msg->axes[3]; // right stick
      curr_pose.position.x +=  (0.001)*msg->axes[4]; // right stick
      curr_pose.position.y += -(0.001)*msg->axes[0]; // left stick
      curr_pose.position.z +=  (0.001)*msg->axes[1]; // left stick

      // LT: rotate counter clockwise
      if(msg->axes[2] == 1)
      {
        theta += (0.05)*msg->axes[2];
        curr_pose.orientation.w = cos(theta);
        curr_pose.orientation.z = sin(theta);
      }
      
      // RT: rotate clockwise
      if(msg->axes[5] == 1)
      {
        theta -= (0.05)*msg->axes[5];
        curr_pose.orientation.w = cos(theta);
        curr_pose.orientation.z = sin(theta);
      }

      // RT change value for applying constraints
      if(msg->buttons[3])
      {
        apply_constraints = !apply_constraints;
        RCLCPP_INFO(LOGGER,
          "Apply constraints value: %d",
          apply_constraints
        );
      }

      // Print updated pose
      if (new_pose != curr_pose) 
      {
        RCLCPP_INFO(
          LOGGER,
          "Position: x = %f, y = %f, z = %f \n Quaternion: x = %f, y = %f, z = %f, w = %f", 
          new_pose.position.x, 
          new_pose.position.y, 
          new_pose.position.z,
          new_pose.orientation.x, 
          new_pose.orientation.y, 
          new_pose.orientation.z,
          new_pose.orientation.w);
        this->publish_marker(new_pose);
      }
    }

    /**
     * Hanldes controller input from the topic /commands/joy_raw
     * @msg: controller input
     */
    void raw_joy_handler(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      static bool flip_ee_x_cmd = false;
      static bool flip_ee_x_cmd_last_state = false;
      static bool flip_torque_cmd = true;
      static bool flip_torque_cmd_last_state = true;
      static double time_start;
      static bool timer_started = false;
      ArmJoy joy_cmd;

      // Check if the torque_cmd should be flipped
      if (msg->buttons.at(btn_map["TORQUE_ENABLE"]) == 1 
          && !flip_torque_cmd_last_state)
      {
        flip_torque_cmd = true;
        joy_cmd.torque_cmd = ArmJoy::TORQUE_ON;
      }
      else if (msg->buttons.at(btn_map["TORQUE_ENABLE"]) == 1 
                && flip_torque_cmd_last_state) 
      {
        time_start = this->get_clock()->now().seconds();
        timer_started = true;
      }
      else if (msg->buttons.at(btn_map["TORQUE_ENABLE"]) == 0)
      {
        if (timer_started && 
            this->get_clock()->now().seconds() - time_start > 3.0) 
        {
          joy_cmd.torque_cmd = ArmJoy::TORQUE_OFF;
          flip_torque_cmd = false;
        }
        flip_torque_cmd_last_state = flip_torque_cmd;
        timer_started = false;
      }

      // Check the speed_cmd
      if (msg->axes.at(btn_map["SPEED"]) == 1)
      {
        joy_cmd.speed_cmd = ArmJoy::SPEED_INC;
      }
      else if (msg->axes.at(btn_map["SPEED"]) == -1)
      {
        joy_cmd.speed_cmd = ArmJoy::SPEED_DEC;
      }

      // Check the speed_toggle_cmd
      if (msg->axes.at(btn_map["SPEED_TYPE"]) == 1)
      {
        joy_cmd.speed_toggle_cmd = ArmJoy::SPEED_COARSE;
      }
      else if (msg->axes.at(btn_map["SPEED_TYPE"]) == -1)
      {
        joy_cmd.speed_toggle_cmd = ArmJoy::SPEED_FINE;
      }

      // Check if the ee_x_cmd should be flipped
      if (msg->buttons.at(btn_map["FLIP_EE_X"]) == 1
            && !flip_ee_x_cmd_last_state)
      {
        flip_ee_x_cmd = true;
      }
      else if (msg->buttons.at(btn_map["FLIP_EE_X"]) == 1
                && flip_ee_x_cmd_last_state)
      {
        flip_ee_x_cmd = false;
      }
      else if (msg->buttons.at(btn_map["FLIP_EE_X"]) == 0) {
        flip_ee_x_cmd_last_state = flip_ee_x_cmd;
      }

      // Check the ee_x_cmd
      if (msg->axes.at(btn_map["EE_X"]) >= threshold && !flip_ee_x_cmd)
      {
        joy_cmd.ee_x_cmd = ArmJoy::EE_X_INC;
      }
      else if (msg->axes.at(btn_map["EE_X"]) <= -threshold && !flip_ee_x_cmd)
      {
        joy_cmd.ee_x_cmd = ArmJoy::EE_X_DEC;
      }
      else if (msg->axes.at(btn_map["EE_X"]) >= threshold && flip_ee_x_cmd)
      {
        joy_cmd.ee_x_cmd = ArmJoy::EE_X_DEC;
      }
      else if (msg->axes.at(btn_map["EE_X"]) <= -threshold && flip_ee_x_cmd)
      {
        joy_cmd.ee_x_cmd = ArmJoy::EE_X_INC;
      }

      // Check the ee_z_cmd
      if (msg->axes.at(btn_map["EE_Z"]) >= threshold)
      {
        joy_cmd.ee_z_cmd = ArmJoy::EE_Z_INC;
      }
      else if (msg->axes.at(btn_map["EE_Z"]) <= -threshold)
      {
        joy_cmd.ee_z_cmd = ArmJoy::EE_Z_DEC;
      }

      // Check the ee_pitch_cmd
      if (msg->axes.at(btn_map["EE_PITCH"]) >= threshold)
      {
        joy_cmd.ee_pitch_cmd = ArmJoy::EE_PITCH_UP;
      }
      else if (msg->axes.at(btn_map["EE_PITCH"]) <= -threshold) {
        joy_cmd.ee_pitch_cmd = ArmJoy::EE_PITCH_DOWN;
      }

      // Check the waist_cmd
      if (msg->buttons.at(btn_map["WAIST_CCW"]) == 1)
      {
        joy_cmd.waist_cmd = ArmJoy::WAIST_CCW;
      }
      else if (msg->buttons.at(btn_map["WAIST_CW"]) == 1)
      {
        joy_cmd.waist_cmd = ArmJoy::WAIST_CW;
      }

      // Check pose_cmd
      if (msg->buttons.at(btn_map["HOME_POSE"]) == 1)
      {
        joy_cmd.pose_cmd = ArmJoy::HOME_POSE;
      }
      else if (msg->buttons.at(btn_map["SLEEP_POSE"]) == 1)
      {
        joy_cmd.pose_cmd = ArmJoy::SLEEP_POSE;
      }
      else if (msg->buttons.at(btn_map["START_LIFT"]) == 1)
      {
        joy_cmd.pose_cmd = ArmJoy::START_LIFT;
      }
      else if (msg->buttons.at(btn_map["LIFT"]) == 1)
      {
        joy_cmd.pose_cmd = ArmJoy::LIFT;
      }
      else if (msg->buttons.at(btn_map["MOVE_FORWARD"]) == 1)
      {
        joy_cmd.pose_cmd = 70;
      }

      // Only publish a ArmJoy message if any of the following fields have changed.
      if (
        !((prev_joy_cmd.ee_x_cmd == joy_cmd.ee_x_cmd) &&
        (prev_joy_cmd.ee_z_cmd == joy_cmd.ee_z_cmd) &&
        (prev_joy_cmd.ee_pitch_cmd == joy_cmd.ee_pitch_cmd) &&
        (prev_joy_cmd.waist_cmd == joy_cmd.waist_cmd) &&
        (prev_joy_cmd.pose_cmd == joy_cmd.pose_cmd) &&
        (prev_joy_cmd.speed_cmd == joy_cmd.speed_cmd) &&
        (prev_joy_cmd.speed_toggle_cmd == joy_cmd.speed_toggle_cmd) &&
        (prev_joy_cmd.torque_cmd == joy_cmd.torque_cmd)))
      {
        pub_joy_cmd->publish(joy_cmd);
      }
      this->prev_joy_cmd = joy_cmd;
    }

    /**
     * Publishes the target pose to the topic /joy_target_pose
     */
    void publish_pose(Pose new_pose, bool use_constraints, std::string pose_name)
    {
      auto msg = ConstrainedPose();
      msg.pose = new_pose;
      msg.use_plane_constraint = use_constraints;
      msg.pose_name = pose_name;
      pose_publisher_->publish(msg);
    }

    /**
     * Publishes the location of the target pose as a marker in RViz.
     */
    void publish_marker(Pose new_pose)
    {
      auto marker = Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.id      = 0;
      marker.type    = Marker::ARROW;
      marker.action  = Marker::ADD;
      marker.scale.x = 0.03;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.pose    = new_pose;
      goal_pose_joy_publisher_->publish(marker);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyInputHandler>());
  rclcpp::shutdown();
  return 0;
}