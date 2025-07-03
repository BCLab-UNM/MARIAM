#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <map>
#include <vector>
#include <string>
#include <memory>

class AnalyticalParameterizer : public rclcpp::Node
{
public:
    AnalyticalParameterizer() : Node("centralized_parameterizer")
    {
        // Declare and get robot_namespace parameter
        this->declare_parameter<std::vector<std::string>>("robot_namespace", std::vector<std::string>());
        robot_namespaces_ = this->get_parameter("robot_namespace").as_string_array();

        if (robot_namespaces_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No robot namespaces provided!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Initializing for %zu robot namespaces", robot_namespaces_.size());

        // Initialize data structures and subscribers for each namespace
        for (const auto& ns : robot_namespaces_) {
            initializeNamespace(ns);
        }

        // Create global publishers (not namespaced)
        mass_pub_ = this->create_publisher<std_msgs::msg::Float64>("/admittance_control/mass", 10);
        damping_pub_ = this->create_publisher<std_msgs::msg::Float64>("/admittance_control/damping", 10);
        stiffness_pub_ = this->create_publisher<std_msgs::msg::Float64>("/admittance_control/stiffness", 10);
        pose_updater_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/admittance_control/px100_virtual_pose_updater", 10);

        // Create timer for publishing at 500Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), // 500Hz = 2ms period
            std::bind(&AnalyticalParameterizer::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Analytical Parameterizer initialized");
    }

private:
    struct NamespaceData {
        // TF handling
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        
        // Stored covariance matrices
        std::array<double, 36> amcl_covariance;
        std::array<double, 36> odom_covariance;
        
        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        

        
        // Distance between robot bases (calculated in timer)
        double inter_robot_distance = 0.0;
        bool inter_robot_distance_valid = false;
        
        // Base link position in odom frame
        geometry_msgs::msg::Point base_position;
        bool base_position_valid = false;
        
        NamespaceData() {
            // Initialize covariance matrices to zero
            amcl_covariance.fill(0.0);
            odom_covariance.fill(0.0);
        }
    };

    void initializeNamespace(const std::string& ns)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing namespace: %s", ns.c_str());
        
        auto& data = namespace_data_[ns];
        
        // Initialize TF buffer and listener for this namespace
        data.tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        data.tf_listener = std::make_shared<tf2_ros::TransformListener>(*data.tf_buffer);
        
        // Create subscribers
        std::string amcl_topic = ns + "/amcl_pose";
        std::string odom_topic = ns + "/odom/filtered";
        
        data.amcl_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            amcl_topic, 10,
            [this, ns](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                this->amclCallback(msg, ns);
            }
        );
        
        data.odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            [this, ns](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odomCallback(msg, ns);
            }
        );
        

        
        RCLCPP_INFO(this->get_logger(), "Namespace %s initialized successfully", ns.c_str());
    }
    
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, const std::string& ns)
    {
        auto& data = namespace_data_[ns];
        
        // Store covariance matrix
        for (size_t i = 0; i < 36; ++i) {
            data.amcl_covariance[i] = msg->pose.covariance[i];
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Updated AMCL covariance for namespace: %s", ns.c_str());
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& ns)
    {
        auto& data = namespace_data_[ns];
        
        // Store covariance matrix
        for (size_t i = 0; i < 36; ++i) {
            data.odom_covariance[i] = msg->pose.covariance[i];
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Updated odometry covariance for namespace: %s", ns.c_str());
    }
    
    void updateBasePosition(const std::string& ns)
    {
        auto& data = namespace_data_[ns];
        
        try {
            // Look up transform from odom to base_link for this namespace
            std::string target_frame = ns + "/odom";
            std::string source_frame = ns + "/base_link";
            
            geometry_msgs::msg::TransformStamped transform = data.tf_buffer->lookupTransform(
                target_frame, source_frame, tf2::TimePointZero
            );
            
            // Store base link position in odom frame
            data.base_position.x = transform.transform.translation.x;
            data.base_position.y = transform.transform.translation.y;
            data.base_position.z = transform.transform.translation.z;
            data.base_position_valid = true;
            
            RCLCPP_DEBUG(this->get_logger(), "Base position for %s: (%.3f, %.3f, %.3f)", 
                ns.c_str(), data.base_position.x, data.base_position.y, data.base_position.z);
            
        } catch (const tf2::TransformException& ex) {
            data.base_position_valid = false;
            RCLCPP_DEBUG(this->get_logger(), "Could not get transform for %s: %s", ns.c_str(), ex.what());
        }
    }
    
    void calculateInterRobotDistance()
    {
        // Need at least 2 robots to calculate distance
        if (robot_namespaces_.size() < 2) {
            return;
        }
        
        // Get positions of first two robots (assuming we want distance between first two)
        const auto& ns1 = robot_namespaces_[0];
        const auto& ns2 = robot_namespaces_[1];
        
        const auto& data1 = namespace_data_.at(ns1);
        const auto& data2 = namespace_data_.at(ns2);
        
        // Check if both positions are valid
        if (!data1.base_position_valid || !data2.base_position_valid) {
            // Mark all robots' distances as invalid if we can't calculate
            for (const auto& ns : robot_namespaces_) {
                namespace_data_[ns].inter_robot_distance_valid = false;
            }
            return;
        }
        
        // Offset the second robot's position by 1 meter in x direction
        geometry_msgs::msg::Point pos1 = data1.base_position;
        geometry_msgs::msg::Point pos2 = data2.base_position;
        pos2.x += 1.0;  // Offset by 1 meter in x direction
        
        // Calculate distance between the two positions (ignoring frame differences)
        double dx = pos1.x - pos2.x;
        double dy = pos1.y - pos2.y;
        double dz = pos1.z - pos2.z;
        
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // Store the same distance for all robots (since it's inter-robot distance)
        for (const auto& ns : robot_namespaces_) {
            auto& data = namespace_data_[ns];
            data.inter_robot_distance = distance;
            data.inter_robot_distance_valid = true;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Inter-robot distance: %.3f meters", distance);
    }
    
    std::vector<double> extractUncertainty(const std::array<double, 36>& covariance_matrix)
    {
        // Extract pose uncertainty from 6x6 covariance matrix
        // Matrix layout: [x, y, z, roll, pitch, yaw]
        // We want diagonal elements (variances) and take square root for standard deviations
        std::vector<double> uncertainty(6);
        
        // Position uncertainties (x, y, z)
        uncertainty[0] = std::sqrt(std::abs(covariance_matrix[0]));   // x uncertainty
        uncertainty[1] = std::sqrt(std::abs(covariance_matrix[7]));   // y uncertainty  
        uncertainty[2] = std::sqrt(std::abs(covariance_matrix[14]));  // z uncertainty
        
        // Orientation uncertainties (roll, pitch, yaw)
        uncertainty[3] = std::sqrt(std::abs(covariance_matrix[21]));  // roll uncertainty
        uncertainty[4] = std::sqrt(std::abs(covariance_matrix[28]));  // pitch uncertainty
        uncertainty[5] = std::sqrt(std::abs(covariance_matrix[35]));  // yaw uncertainty
        
        return uncertainty;
    }
    
    void timerCallback()
    {
        // Update base positions for all namespaces
        for (const auto& ns : robot_namespaces_) {
            updateBasePosition(ns);
        }
        
        // Calculate inter-robot distance
        calculateInterRobotDistance();
        
        // Extract uncertainties from covariance matrices for all namespaces
        for (const auto& ns : robot_namespaces_) {
            const auto& data = namespace_data_.at(ns);
            
            // Extract AMCL pose uncertainty
            std::vector<double> amcl_uncertainty = extractUncertainty(data.amcl_covariance);
            RCLCPP_DEBUG(this->get_logger(), 
                "AMCL uncertainty for %s - x:%.3f, y:%.3f, z:%.3f, roll:%.3f, pitch:%.3f, yaw:%.3f", 
                ns.c_str(), amcl_uncertainty[0], amcl_uncertainty[1], amcl_uncertainty[2],
                amcl_uncertainty[3], amcl_uncertainty[4], amcl_uncertainty[5]);
            
            // Extract odometry pose uncertainty
            std::vector<double> odom_uncertainty = extractUncertainty(data.odom_covariance);
            RCLCPP_DEBUG(this->get_logger(), 
                "Odom uncertainty for %s - x:%.3f, y:%.3f, z:%.3f, roll:%.3f, pitch:%.3f, yaw:%.3f", 
                ns.c_str(), odom_uncertainty[0], odom_uncertainty[1], odom_uncertainty[2],
                odom_uncertainty[3], odom_uncertainty[4], odom_uncertainty[5]);
        }
        
        // TODO: Use the extracted uncertainties and base distances to calculate
        // and publish mass, damping, stiffness parameters and pose updates
        
        RCLCPP_DEBUG(this->get_logger(), "Timer callback executed");
    }
    
    // Global publishers (not namespaced)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mass_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr damping_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stiffness_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_updater_pub_;
    
    // Member variables
    std::vector<std::string> robot_namespaces_;
    std::map<std::string, NamespaceData> namespace_data_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnalyticalParameterizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}