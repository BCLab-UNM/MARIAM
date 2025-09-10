#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pid_controller.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <memory>
#include <sstream>
#include <fstream>
#include <iomanip>  // Added for std::setprecision
#include <cstdlib>


struct DataPoint {
  double des_ross_x;
  double des_ross_y;
  double des_ross_theta;

  double act_ross_x;
  double act_ross_y;
  double act_ross_theta;
  
  double des_mon_x;
  double des_mon_y;
  double des_mon_theta;
  
  double act_mon_x;
  double act_mon_y;
  double act_mon_theta;
  
  double act_payload_x;
  double act_payload_y;
  double act_payload_z;
  double act_payload_theta;
  
  // TODO: add desired payload pose?

  double ross_cmd_vel_linear_x;
  double ross_cmd_vel_angular_z;
  double monica_cmd_vel_linear_x;
  double monica_cmd_vel_angular_z;
  double dt;
  rclcpp::Time ros_time;
};


class CoopTrajPID : public rclcpp::Node
{
public:
  CoopTrajPID() : Node("coop_traj_pid") {
    marker_to_base.setIdentity();
    marker_to_base.setOrigin(tf2::Vector3(0.23, -0.075, -0.06));
    marker_to_base.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // change QoS settings
    auto qos_profile = rclcpp::QoS(10);
    qos_profile.best_effort();
    qos_profile.durability_volatile();

    ross_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ross/cmd_vel", qos_profile);
    
    monica_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/monica/cmd_vel", qos_profile);

    bases_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/desired_poses",
      qos_profile,
      std::bind(&CoopTrajPID::bases_callback, this, std::placeholders::_1)
    );
    
    base1_actual_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/world_ross_pose",
      qos_profile,
      std::bind(&CoopTrajPID::base1_actual_callback, this, std::placeholders::_1)
    );
    
    base2_actual_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/world_monica_pose",
      qos_profile,
      std::bind(&CoopTrajPID::base2_actual_callback, this, std::placeholders::_1)
    );

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter("max_linear_velocity", 0.2);
    this->declare_parameter("max_angular_velocity", 0.4);

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
    
    // Create timer for control loop
    this->declare_parameter("dt", 0.01); // default 10ms
    dt_ = this->get_parameter("dt").as_double();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&CoopTrajPID::control_loop, this)
    );

    this->declare_parameter("trial_name", "test_trial");
    std::string trial_name = this->get_parameter("trial_name").as_string();
  }

  // Add this destructor
  ~CoopTrajPID() {
    RCLCPP_INFO(this->get_logger(), "Node shutting down, writing CSV data...");
    write_to_csv();
  }


private:

  void write_to_csv() {
    std::string trial_name = this->get_parameter("trial_name").as_string();

    // Check if trial_name is empty
    if (trial_name.empty()) {
      // if so, skip writing CSV
      RCLCPP_INFO(this->get_logger(), "Trial name is empty, skipping CSV write. Collected %zu data points.", data_points_.size());
      return;
    }
    
    try {
      rcpputils::fs::path file_path;
      
      // Check if trial_name contains a path separator (/ or ~)
      if (trial_name.find('/') != std::string::npos || trial_name.front() == '~') {
        // trial_name is a full or relative path
        if (trial_name.front() == '~') {
          // Expand ~ to home directory
          const char* home_dir = std::getenv("HOME");
          if (home_dir) {
            trial_name = std::string(home_dir) + trial_name.substr(1);
          }
        }
        
        // Use trial_name as the complete directory path
        rcpputils::fs::path data_dir = rcpputils::fs::path(trial_name);
        
        // Create the directory if it doesn't exist
        if (!rcpputils::fs::create_directories(data_dir)) {
          // create_directories returns false if directory already exists, check if it actually exists
          if (!rcpputils::fs::exists(data_dir)) {
            // if it does not exist, log error and return
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", data_dir.string().c_str());
            return;
          }
        }
        RCLCPP_INFO(this->get_logger(), "Using directory: %s", data_dir.string().c_str());
        
        // Construct the full file path - Fixed string concatenation
        std::string filename = "trajectory_data.csv";
        file_path = data_dir / filename;
      }
      
      else {
        // trial_name is just a name, use with data_directory parameter
        // Check if data_directory parameter exists, if not create a default
        if (!this->has_parameter("data_directory")) {
          const char* home_dir = std::getenv("HOME");
          std::string default_data_dir = std::string(home_dir ? home_dir : "/tmp") + "/.arm_controller/data/";
          this->declare_parameter("data_directory", default_data_dir);
        }
        
        std::string data_directory = this->get_parameter("data_directory").as_string();
        rcpputils::fs::path data_dir = rcpputils::fs::path(data_directory) / trial_name;
        
        // Create the directory if it doesn't exist
        if (!rcpputils::fs::create_directories(data_dir)) {
          // create_directories returns false if directory already exists, check if it actually exists
          if (!rcpputils::fs::exists(data_dir)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", data_dir.string().c_str());
            return;
          }
        }
        RCLCPP_INFO(this->get_logger(), "Using directory: %s", data_dir.string().c_str());
        
        // Construct the full file path
        file_path = data_dir / "trajectory_data.csv";
      }
      
      std::string file_name = file_path.string();
      RCLCPP_INFO(this->get_logger(), "Writing CSV to: %s", file_name.c_str());
    
    // Create ofstream object with the filename
    std::ofstream file(file_name);

    // Don't call file.open() - constructor already opened it
    // file.open();  // REMOVED - this was causing the issue

    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing: %s", file_name.c_str());
      return;
    }
      
    // Write header
    file << "des_ross_x,des_ross_y,des_ross_theta,des_mon_x,des_mon_y,des_mon_theta,act_ross_x,act_ross_y,act_ross_theta,act_mon_x,act_mon_y,act_mon_theta,act_payload_x,act_payload_y,act_payload_theta,ross_cmd_vel_linear_x,ross_cmd_vel_angular_z,monica_cmd_vel_linear_x,monica_cmd_vel_angular_z,dt,ros_time_sec,ros_time_ns\n";

    // Write data with proper precision
    file << std::fixed << std::setprecision(6);
    for (const auto& point : data_points_) {
      file << point.des_ross_x << "," 
            << point.des_ross_y << "," 
            << point.des_ross_theta << "," 
            
            << point.des_mon_x << "," 
            << point.des_mon_y << "," 
            << point.des_mon_theta << "," 
            
            << point.act_ross_x << "," 
            << point.act_ross_y << "," 
            << point.act_ross_theta << "," 
            
            << point.act_mon_x << "," 
            << point.act_mon_y << "," 
            << point.act_mon_theta << "," 
            
            << point.act_payload_x << "," 
            << point.act_payload_y << "," 
            << point.act_payload_z << "," 
            << point.act_payload_theta << "," 
            
            << point.ross_cmd_vel_linear_x << "," 
            << point.ross_cmd_vel_angular_z << "," 
            
            << point.monica_cmd_vel_linear_x << "," 
            << point.monica_cmd_vel_angular_z << "," 
            
            << point.dt << "," 
            << point.ros_time.seconds() << "," 
            << point.ros_time.nanoseconds() << "\n";
    }
    
    file.close();
    RCLCPP_INFO(this->get_logger(), "Data written to %s (%zu points)", file_name.c_str(), data_points_.size());
    
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error writing CSV file: %s", e.what());
    }
  }

  geometry_msgs::msg::Pose transform_pose(const geometry_msgs::msg::Pose &pose) {
    // Convert geometry_msgs::msg::Pose to tf2::Transform
    tf2::Transform pose_tf;
    pose_tf.setOrigin(
      tf2::Vector3(
        pose.position.x, pose.position.y, pose.position.z));
    pose_tf.setRotation(
      tf2::Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w));

    // Apply the transformation
    tf2::Transform transformed_tf = pose_tf * marker_to_base;

    // Convert back to geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose transformed_pose;
    transformed_pose.position.x = transformed_tf.getOrigin().x();
    transformed_pose.position.y = transformed_tf.getOrigin().y();
    transformed_pose.position.z = transformed_tf.getOrigin().z();
    tf2::Quaternion q = transformed_tf.getRotation();
    transformed_pose.orientation.x = q.x();
    transformed_pose.orientation.y = q.y();
    transformed_pose.orientation.z = q.z();
    transformed_pose.orientation.w = q.w();

    return transformed_pose;
  }

  geometry_msgs::msg::TransformStamped lookup_transform(const std::string& target_frame,
                                                        const std::string& source_frame) {
    try {
      // Wait for up to 0.5 seconds for the transform
      return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, std::chrono::milliseconds(500));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
      geometry_msgs::msg::TransformStamped empty;
      return empty;
    }
  }

  // subscriber callbacks
  void base1_actual_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    base1_actual_pose_ = transform_pose(*msg);
  }

  void base2_actual_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    base2_actual_pose_ = transform_pose(*msg);
  }

  void bases_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.size() >= 2) {
      base1_pose_ = msg->poses[0];
      base2_pose_ = msg->poses[1];
      bases_received_ = true;
    }
  }

  double get_yaw(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

  void control_loop() {
    // if no data has been received
    if (!bases_received_) {
      RCLCPP_WARN(this->get_logger(), "No base poses received yet.");
      // Wait until both base poses are getting published
      // publish zero velocities to stop the robots
      auto zero_twist = geometry_msgs::msg::Twist();
      ross_cmd_vel_pub_->publish(zero_twist);
      monica_cmd_vel_pub_->publish(zero_twist);
      return;
    }

    // Compute finite difference velocities (not used currently)
    geometry_msgs::msg::Twist cmd_vel1_ff = compute_feedforward_velocity(base1_pose_, last_base1_desired_pose_, dt_);
    geometry_msgs::msg::Twist cmd_vel2_ff = compute_feedforward_velocity(base2_pose_, last_base2_desired_pose_, dt_);

    // Store last desired poses
    last_base1_desired_pose_ = base1_pose_;
    last_base2_desired_pose_ = base2_pose_;

    // ----------------------------------------------------------------------
    //                           BASE 1 (Ross)
    // ----------------------------------------------------------------------
    // Calculate errors for base 1 (world frame)
    double world_error_x1 = base1_pose_.position.x - base1_actual_pose_.position.x;
    double world_error_y1 = base1_pose_.position.y - base1_actual_pose_.position.y;
    double desired_theta1 = get_yaw(base1_pose_.orientation);
    double actual_theta1 = get_yaw(base1_actual_pose_.orientation);
    desired_theta1 = normalize_angle(desired_theta1);
    actual_theta1 = normalize_angle(actual_theta1);

    // transform errors to the body frame
    double error_x1 = world_error_x1 * cos(actual_theta1) + world_error_y1 * sin(actual_theta1);
    double error_y1 = -world_error_x1 * sin(actual_theta1) + world_error_y1 * cos(actual_theta1);
    double error_theta1 = desired_theta1 - actual_theta1;
    
    // Compute control signals for base 1
    double control_x1 = base1_pid_x_->compute(error_x1, dt_);
    // TODO: make this match the desired architecture
    double control_y1 = base1_pid_y_->compute(error_y1, dt_);
    double theta_d1 = error_theta1 + control_y1;
    double control_theta1 = base1_pid_theta_->compute(theta_d1, dt_);

    // Create Twist message for base 1
    auto cmd_vel1 = geometry_msgs::msg::Twist();
    cmd_vel1.linear.x = control_x1;
    cmd_vel1.angular.z = control_theta1;

    // ----------------------------------------------------------------------
    //                           BASE 2 (Monica)
    // ----------------------------------------------------------------------
    // Calculate errors for base 2 (world frame)
    double world_error_x2 = base2_pose_.position.x - base2_actual_pose_.position.x;
    double world_error_y2 = base2_pose_.position.y - base2_actual_pose_.position.y;
    double desired_theta2 = get_yaw(base2_pose_.orientation);
    // Add pi to make the PID controller think the heading is behind monica
    double actual_theta2 = get_yaw(base2_actual_pose_.orientation) + M_PI;
    // Normalize angles to [-pi, pi]
    desired_theta2 = normalize_angle(desired_theta2);
    actual_theta2 = normalize_angle(actual_theta2);

    // transform errors to the body frame
    double error_x2 = world_error_x2 * cos(actual_theta2) + world_error_y2 * sin(actual_theta2);
    double error_y2 = -world_error_x2 * sin(actual_theta2) + world_error_y2 * cos(actual_theta2);
    double error_theta2 = desired_theta2 - actual_theta2;

    // Compute control signals for base 2
    double control_x2 = base2_pid_x_->compute(error_x2, dt_);
    // TODO: make this match the desired architecture
    double control_y2 = base2_pid_y_->compute(error_y2, dt_);
    double theta_d2 = error_theta2 + control_y2;
    double control_theta2 = base2_pid_theta_->compute(theta_d2, dt_);

    // Create Twist message for base 2
    auto cmd_vel2 = geometry_msgs::msg::Twist();
    cmd_vel2.linear.x = control_x2;
    cmd_vel2.angular.z = control_theta2;

    // Print debug info
    RCLCPP_INFO(this->get_logger(), "----------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Base 1 Desired Pose: x: %.3f, y: %.3f, theta: %.3f", base1_pose_.position.x, base1_pose_.position.y, desired_theta1);
    RCLCPP_INFO(this->get_logger(), "Base 1 Actual Pose: x: %.3f, y: %.3f, theta: %.3f", base1_actual_pose_.position.x, base1_actual_pose_.position.y, actual_theta1);
    RCLCPP_INFO(this->get_logger(), "Base 1 Errors: x: %.3f, y: %.3f, theta: %.3f", error_x1, error_y1, error_theta1);
    RCLCPP_INFO(this->get_logger(), "Base 1 Controls: x: %.3f, theta: %.3f", control_x1, control_theta1);
    RCLCPP_INFO(this->get_logger(), "Base 1 Feedforward: x: %.3f, theta: %.3f", cmd_vel1_ff.linear.x, cmd_vel1_ff.angular.z);
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "Base 2 Desired Pose: x: %.3f, y: %.3f, theta: %.3f", base2_pose_.position.x, base2_pose_.position.y, desired_theta2);
    RCLCPP_INFO(this->get_logger(), "Base 2 Actual Pose: x: %.3f, y: %.3f, theta: %.3f", base2_actual_pose_.position.x, base2_actual_pose_.position.y, actual_theta2);
    RCLCPP_INFO(this->get_logger(), "Base 2 Errors: x: %.3f, y: %.3f, theta: %.3f", error_x2, error_y2, error_theta2);
    RCLCPP_INFO(this->get_logger(), "Base 2 Controls: x: %.3f, theta: %.3f", control_x2, control_theta2);
    RCLCPP_INFO(this->get_logger(), "Base 2 Feedforward: x: %.3f, theta: %.3f", cmd_vel2_ff.linear.x, cmd_vel2_ff.angular.z);

    // Reverse linear velocity for monica
    cmd_vel2.linear.x = -cmd_vel2.linear.x;
    cmd_vel2_ff.linear.x = -cmd_vel2_ff.linear.x;

    // Publish commands
    ross_cmd_vel_pub_->publish(cmd_vel1_ff);
    monica_cmd_vel_pub_->publish(cmd_vel2);

    // Save data point
    DataPoint point;
    point.des_ross_x = base1_pose_.position.x;
    point.des_ross_y = base1_pose_.position.y;
    point.des_ross_theta = desired_theta1;

    point.act_ross_x = base1_actual_pose_.position.x;
    point.act_ross_y = base1_actual_pose_.position.y;
    point.act_ross_theta = actual_theta1;

    point.des_mon_x = base2_pose_.position.x;
    point.des_mon_y = base2_pose_.position.y;
    point.des_mon_theta = desired_theta2;

    point.act_mon_x = base2_actual_pose_.position.x;
    point.act_mon_y = base2_actual_pose_.position.y;
    point.act_mon_theta = actual_theta2;

    geometry_msgs::msg::TransformStamped transform_stamped =
                              lookup_transform("world", "payload");

    point.act_payload_x = transform_stamped.transform.translation.x;
    point.act_payload_y = transform_stamped.transform.translation.y;
    point.act_payload_z = transform_stamped.transform.translation.z;
    point.act_payload_theta = get_yaw(transform_stamped.transform.rotation);

    point.dt = dt_;

    point.ross_cmd_vel_linear_x = cmd_vel1_ff.linear.x;
    point.ross_cmd_vel_angular_z = cmd_vel1_ff.angular.z;
    point.monica_cmd_vel_linear_x = cmd_vel2_ff.linear.x;
    point.monica_cmd_vel_angular_z = cmd_vel2_ff.angular.z;

    point.ros_time = this->now();

    data_points_.push_back(point);
  }

  // Calculates feed forward cmd velocity based on finite differences
  geometry_msgs::msg::Twist compute_feedforward_velocity(
      const geometry_msgs::msg::Pose &current_pose,
      const geometry_msgs::msg::Pose &last_pose, double dt) 
  {
      if (last_pose == geometry_msgs::msg::Pose()) return geometry_msgs::msg::Twist();
      
      double v_x = (current_pose.position.x - last_pose.position.x) / dt;
      
      // Fix angle wrapping
      double current_yaw = get_yaw(current_pose.orientation);
      double last_yaw = get_yaw(last_pose.orientation);
      double angle_diff = normalize_angle(current_yaw - last_yaw);  // Add this line
      double v_theta = angle_diff / dt;
      
      geometry_msgs::msg::Twist twist;
      twist.linear.x = v_x;
      twist.angular.z = v_theta;
      return twist;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr monica_cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ross_cmd_vel_pub_;
  // NOTE: base 1 = Ross, base 2 = Monica
  // Subscriptions for desired base poses
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bases_sub_;
  // Subscriptions for actual base poses
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base1_actual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base2_actual_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // PID controllers for base 1
  std::unique_ptr<PIDController> base1_pid_x_;
  std::unique_ptr<PIDController> base1_pid_y_;
  std::unique_ptr<PIDController> base1_pid_theta_;

  // PID controllers for base 2
  std::unique_ptr<PIDController> base2_pid_x_;
  std::unique_ptr<PIDController> base2_pid_y_;
  std::unique_ptr<PIDController> base2_pid_theta_;

  // Desired poses
  geometry_msgs::msg::Pose base1_pose_;
  geometry_msgs::msg::Pose base2_pose_;
  // Actual poses
  geometry_msgs::msg::Pose base1_actual_pose_;
  geometry_msgs::msg::Pose base2_actual_pose_;

  double dt_;
  bool bases_received_ = false;

  // Store last poses for finite difference
  geometry_msgs::msg::Pose last_base1_desired_pose_;
  geometry_msgs::msg::Pose last_base2_desired_pose_;

  tf2::Transform marker_to_base;
  std::vector<DataPoint> data_points_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoopTrajPID>());
  rclcpp::shutdown();
  return 0;
}
