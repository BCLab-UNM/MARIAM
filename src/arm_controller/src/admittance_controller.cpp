// C++
#include <memory>
#include <sstream>
#include <fstream>
#include <iomanip>  // Added for std::setprecision
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcpputils/filesystem_helper.hpp>

// ROS
    #include <ament_index_cpp/get_package_share_directory.hpp>
    #include <rcpputils/filesystem_helper.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace geometry_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;

struct DataPoint {
  double K;
  Pose target_pose;
  Pose virtual_pose;
  float force;
  rclcpp::Time ros_time;
};


class AdmittanceController : public rclcpp::Node {
  public:
    AdmittanceController() : Node("admittance_controller") {
      auto qos_profile = rclcpp::QoS(10);
      qos_profile.best_effort();
      qos_profile.durability_volatile();

      // =============== creating publishers and subscriptions ===============
      pose_publisher = this->create_publisher<Pose>(
        "px100_target_pose",
        qos_profile
      );

      virtual_pose_sub = this->create_subscription<Pose>(
        "px100_virtual_pose",
        qos_profile,
        std::bind(&AdmittanceController::admittance_control_callback, this, _1)
      );

      force_sub = this->create_subscription<Float32>(
        "force",
        qos_profile,
        std::bind(&AdmittanceController::force_callback, this, _1)
      );

      mass_sub = this->create_subscription<Float64>(
        "admittance_control/mass",
        qos_profile,
        std::bind(&AdmittanceController::mass_callback, this, _1)
      );
      
      damping_sub = this->create_subscription<Float64>(
        "admittance_control/damping",
        qos_profile,
        std::bind(&AdmittanceController::damping_callback, this, _1)
      );
      
      stiffness_sub = this->create_subscription<Float64>(
        "admittance_control/stiffness",
        qos_profile,
        std::bind(&AdmittanceController::stiffness_callback, this, _1)
      );

      // =============== creating parameters ===============
      // declaring parameters for the initial values of
      // mass, damping, and stiffness
      this->declare_parameter("mass",       5.0);
      this->declare_parameter("damping",   10.0);
      this->declare_parameter("stiffness", 15.0);
      this->declare_parameter("trial_name", "");

      // storing the initial values into variables
      this->get_parameter("mass",      mass);
      this->get_parameter("damping",   damping);
      this->get_parameter("stiffness", stiffness);
    }

    void write_to_csv() {
      // Check if trial_name is empty, if so, skip writing CSV
      std::string trial_name = this->get_parameter("trial_name").as_string();
      if (trial_name.empty()) {
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
              RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", data_dir.string().c_str());
              return;
            }
          }
          RCLCPP_INFO(this->get_logger(), "Using directory: %s", data_dir.string().c_str());
          
          // Construct the full file path
          file_path = data_dir / "admittance_data.csv";
        } else {
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
          file_path = data_dir / "admittance_data.csv";
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
      file << "t_p_x,t_p_y,t_p_z,t_w,t_x,t_y,t_z,v_p_x,v_p_y,v_p_z,v_w,v_x,v_y,v_z,force,stiffness,ros_time_sec,ros_time_ns\n";
      
      // Write data with proper precision
      file << std::fixed << std::setprecision(6);
      for (const auto& point : data_points_) {
        file << point.target_pose.position.x << "," 
             << point.target_pose.position.y << "," 
             << point.target_pose.position.z << "," 
             << point.target_pose.orientation.w << "," 
             << point.target_pose.orientation.x << "," 
             << point.target_pose.orientation.y << "," 
             << point.target_pose.orientation.z << "," 
             << point.virtual_pose.position.x << ","
             << point.virtual_pose.position.y << ","
             << point.virtual_pose.position.z << ","
             << point.virtual_pose.orientation.w << ","
             << point.virtual_pose.orientation.x << ","
             << point.virtual_pose.orientation.y << ","
             << point.virtual_pose.orientation.z << ","
             << point.force << "," 
             << point.K << ","
             << point.ros_time.seconds() << ","
             << point.ros_time.nanoseconds() << "\n";
      }
      
      file.close();
      RCLCPP_INFO(this->get_logger(), "Data written to %s (%zu points)", file_name.c_str(), data_points_.size());
      
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error writing CSV file: %s", e.what());
      }
    }


  private:
    rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
    rclcpp::Subscription<Pose>::SharedPtr virtual_pose_sub;
    rclcpp::Subscription<Float32>::SharedPtr force_sub;
    rclcpp::Subscription<Float64>::SharedPtr mass_sub;
    rclcpp::Subscription<Float64>::SharedPtr damping_sub;
    rclcpp::Subscription<Float64>::SharedPtr stiffness_sub;

    // vector for saving stuff
    std::vector<DataPoint> data_points_;
    
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
      
      DataPoint p;

      p.K = stiffness;
      p.target_pose = new_msg;
      p.virtual_pose = *msg;
      p.force = force_reading;
      p.ros_time = this->get_clock()->now();
      data_points_.push_back(p);
    }

    void force_callback(const Float32::SharedPtr msg) {
      force_reading = msg->data;
    }

    void mass_callback(const Float64::SharedPtr msg) {
      RCLCPP_DEBUG(LOGGER, "Updating mass from %.2f to %.2f", mass, msg->data);
      mass = msg->data;
    }
    
    void damping_callback(const Float64::SharedPtr msg) {
      RCLCPP_DEBUG(LOGGER, "Updating damping from %.2f to %.2f", damping, msg->data);
      damping = msg->data;
    }
    
    void stiffness_callback(const Float64::SharedPtr msg) {
      RCLCPP_DEBUG(LOGGER, "Updating stiffness from %.2f to %.2f", stiffness, msg->data);
      stiffness = msg->data;
    }

    
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdmittanceController>();
  rclcpp::spin(node);

  node->write_to_csv();

  rclcpp::shutdown();
  return 0;
}