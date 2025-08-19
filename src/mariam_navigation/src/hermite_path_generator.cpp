// Cubic Hermite interpolation for a single segment#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <memory>

class HermiteSplinePlanner : public rclcpp::Node
{
public:
    HermiteSplinePlanner() : Node("hermite_spline_planner"), sample_distance_(0.3)
    {
        // Subscribers
        monica_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_monica_pose", 10, 
            std::bind(&HermiteSplinePlanner::monicaPoseCallback, this, std::placeholders::_1));
        
        ross_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/world_ross_pose", 10,
            std::bind(&HermiteSplinePlanner::rossPoseCallback, this, std::placeholders::_1));

        // Publishers
        monica_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/monica_path", 10);
        ross_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ross_path", 10);

        RCLCPP_INFO(this->get_logger(), "Hermite Spline Planner initialized. Waiting for poses...");
    }

private:
    struct Waypoint {
        tf2::Vector3 position;
        tf2::Vector3 tangent;
    };

    // Class members
    double sample_distance_;
    bool monica_pose_received_ = false;
    bool ross_pose_received_ = false;
    geometry_msgs::msg::Pose monica_pose_;
    geometry_msgs::msg::Pose ross_pose_;

    // ROS2 components
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr monica_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ross_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr monica_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ross_path_pub_;

    void monicaPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        monica_pose_ = *msg;
        monica_pose_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received Monica pose");
        checkAndProcess();
    }

    void rossPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        ross_pose_ = *msg;
        ross_pose_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received Ross pose");
        checkAndProcess();
    }

    void checkAndProcess()
    {
        if (monica_pose_received_ && ross_pose_received_) {
            generateAndPublishPaths();
        }
    }

    tf2::Vector3 extractXAxis(const geometry_msgs::msg::Pose& pose)
    {
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        tf2::Matrix3x3 rotation_matrix(q);
        return rotation_matrix.getColumn(0); // x-axis direction
    }

    tf2::Vector3 poseToVector3(const geometry_msgs::msg::Pose& pose)
    {
        return tf2::Vector3(pose.position.x, pose.position.y, pose.position.z);
    }

    geometry_msgs::msg::Pose vector3AndQuatToPose(const tf2::Vector3& position, const tf2::Quaternion& orientation)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        pose.orientation = tf2::toMsg(orientation);
        return pose;
    }

    tf2::Quaternion vectorToQuaternion(const tf2::Vector3& direction)
    {
        // Create quaternion from direction vector (assuming direction is the x-axis)
        tf2::Vector3 x_axis = direction.normalized();
        tf2::Vector3 z_axis(0, 0, 1); // Assume z points up
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
        z_axis = x_axis.cross(y_axis);

        tf2::Matrix3x3 rotation_matrix(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z()
        );

        tf2::Quaternion quat;
        rotation_matrix.getRotation(quat);
        return quat;
    }

    double calculateSegmentLength(const Waypoint& p0, const Waypoint& p1, int resolution = 1000)
    {
        double total_length = 0.0;
        tf2::Vector3 prev_pos = p0.position;
        
        for (int i = 1; i <= resolution; i++) {
            double t = static_cast<double>(i) / static_cast<double>(resolution);
            geometry_msgs::msg::Pose pose = interpolateHermite(p0, p1, t);
            tf2::Vector3 current_pos(pose.position.x, pose.position.y, pose.position.z);
            total_length += (current_pos - prev_pos).length();
            prev_pos = current_pos;
        }
        
        return total_length;
    }

    double findParameterForDistance(const Waypoint& p0, const Waypoint& p1, double target_distance, double segment_length)
    {
        if (target_distance <= 0.0) return 0.0;
        if (target_distance >= segment_length) return 1.0;

        // Binary search to find parameter t that gives us the target distance
        double low = 0.0, high = 1.0;
        double tolerance = 1e-6;
        
        while (high - low > tolerance) {
            double mid = (low + high) * 0.5;
            
            // Calculate arc length from start to mid point
            double length = 0.0;
            tf2::Vector3 prev_pos = p0.position;
            int resolution = 100;
            
            for (int i = 1; i <= static_cast<int>(mid * resolution); i++) {
                double t = static_cast<double>(i) / static_cast<double>(resolution);
                geometry_msgs::msg::Pose pose = interpolateHermite(p0, p1, t);
                tf2::Vector3 current_pos(pose.position.x, pose.position.y, pose.position.z);
                length += (current_pos - prev_pos).length();
                prev_pos = current_pos;
            }
            
            if (length < target_distance) {
                low = mid;
            } else {
                high = mid;
            }
        }
        
        return (low + high) * 0.5;
    }
    geometry_msgs::msg::Pose interpolateHermite(const Waypoint& p0, const Waypoint& p1, double t)
    {
        // Hermite basis functions
        double h00 = 2*t*t*t - 3*t*t + 1;
        double h10 = t*t*t - 2*t*t + t;
        double h01 = -2*t*t*t + 3*t*t;
        double h11 = t*t*t - t*t;

        // Calculate position
        tf2::Vector3 position = h00 * p0.position + h10 * p0.tangent + h01 * p1.position + h11 * p1.tangent;

        // Calculate tangent at this point for orientation
        double dh00_dt = 6*t*t - 6*t;
        double dh10_dt = 3*t*t - 4*t + 1;
        double dh01_dt = -6*t*t + 6*t;
        double dh11_dt = 3*t*t - 2*t;

        tf2::Vector3 tangent = dh00_dt * p0.position + dh10_dt * p0.tangent + dh01_dt * p1.position + dh11_dt * p1.tangent;
        
        // Convert tangent to orientation
        tf2::Quaternion orientation = vectorToQuaternion(tangent);

        return vector3AndQuatToPose(position, orientation);
    }

    std::vector<geometry_msgs::msg::Pose> sampleSplineSegments(const std::vector<Waypoint>& waypoints, 
                                                               int start_segment, int end_segment)
    {
        std::vector<geometry_msgs::msg::Pose> sampled_poses;
        
        // Calculate total path length
        std::vector<double> segment_lengths;
        double total_length = 0.0;
        
        for (int seg = start_segment; seg <= end_segment; seg++) {
            double length = calculateSegmentLength(waypoints[seg], waypoints[seg + 1]);
            segment_lengths.push_back(length);
            total_length += length;
        }
        
        // Calculate number of samples needed
        int total_samples = static_cast<int>(std::ceil(total_length / sample_distance_)) + 1;
        
        // Sample at fixed distances
        double current_distance = 0.0;
        int current_segment = start_segment;
        double distance_in_current_segment = 0.0;
        
        for (int i = 0; i < total_samples; i++) {
            double target_distance = static_cast<double>(i) * sample_distance_;
            
            // Find which segment contains this target distance
            while (current_segment <= end_segment && 
                   target_distance > current_distance + segment_lengths[current_segment - start_segment]) {
                current_distance += segment_lengths[current_segment - start_segment];
                current_segment++;
            }
            
            // If we've gone past the last segment, add the final waypoint and break
            if (current_segment > end_segment) {
                geometry_msgs::msg::Pose final_pose = interpolateHermite(waypoints[end_segment], waypoints[end_segment + 1], 1.0);
                sampled_poses.push_back(final_pose);
                break;
            }
            
            // Calculate distance within current segment
            distance_in_current_segment = target_distance - current_distance;
            
            // Find parameter t for this distance within the segment
            double t = findParameterForDistance(waypoints[current_segment], waypoints[current_segment + 1], 
                                              distance_in_current_segment, segment_lengths[current_segment - start_segment]);
            
            geometry_msgs::msg::Pose interpolated_pose = interpolateHermite(waypoints[current_segment], waypoints[current_segment + 1], t);
            sampled_poses.push_back(interpolated_pose);
        }
        
        // Ensure the final point is exactly at the last waypoint
        if (!sampled_poses.empty()) {
            geometry_msgs::msg::Pose final_pose = interpolateHermite(waypoints[end_segment], waypoints[end_segment + 1], 1.0);
            sampled_poses.back() = final_pose;
        }

        return sampled_poses;
    }

    nav_msgs::msg::Path createPath(const std::vector<geometry_msgs::msg::Pose>& poses, const std::string& frame_id)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = frame_id;
        path.header.stamp = this->get_clock()->now();

        for (const auto& pose : poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path.header;
            pose_stamped.pose = pose;
            path.poses.push_back(pose_stamped);
        }

        return path;
    }

    void generateAndPublishPaths()
    {
        RCLCPP_INFO(this->get_logger(), "Generating paths using cubic Hermite splines...");

        // Create waypoints
        std::vector<Waypoint> waypoints(4);
        
        // Waypoint 1: Ross's pose
        waypoints[0].position = poseToVector3(ross_pose_);
        waypoints[0].tangent = extractXAxis(ross_pose_);

        // Waypoint 2: Monica's pose
        waypoints[1].position = poseToVector3(monica_pose_);
        waypoints[1].tangent = extractXAxis(monica_pose_);

        // Waypoint 3: 2m in positive x direction from waypoint 2
        waypoints[2].position = waypoints[1].position + tf2::Vector3(2.0, 2.0, 0.0);
        waypoints[2].tangent = tf2::Vector3(0.0, 1.0, 0.0); // Pointing in positive x

        // Waypoint 4: 1m in positive x direction from waypoint 3
        waypoints[3].position = waypoints[2].position + tf2::Vector3(0.0, 1.0, 0.0);
        waypoints[3].tangent = tf2::Vector3(0.0, 1.0, 0.0); // Pointing in positive x

        // Scale tangents for smoother curves
        for (int i = 0; i < waypoints.size(); i++) waypoints[i].tangent *= 3;

        // Create sub-splines
        // Ross path: segments 1 and 2 (waypoints 1->2->3)
        std::vector<geometry_msgs::msg::Pose> ross_poses = sampleSplineSegments(waypoints, 0, 1);
        nav_msgs::msg::Path ross_path = createPath(ross_poses, "world");

        // Monica path: segments 2 and 3 (waypoints 2->3->4)  
        std::vector<geometry_msgs::msg::Pose> monica_poses = sampleSplineSegments(waypoints, 1, 2);
        nav_msgs::msg::Path monica_path = createPath(monica_poses, "world");

        // Publish paths
        ross_path_pub_->publish(ross_path);
        monica_path_pub_->publish(monica_path);

        RCLCPP_INFO(this->get_logger(), "Published Ross path with %zu poses", ross_path.poses.size());
        RCLCPP_INFO(this->get_logger(), "Published Monica path with %zu poses", monica_path.poses.size());
        RCLCPP_INFO(this->get_logger(), "Path planning complete. Shutting down...");

        // Shutdown after publishing
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HermiteSplinePlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}