#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace geometry_msgs::msg;
using namespace visualization_msgs::msg;
using std::placeholders::_1;

class AdmittanceRvizMarkers : public rclcpp::Node {
public:
  AdmittanceRvizMarkers() : Node("admittance_rviz_markers") {
    // Create separate publishers for the target and virtual RViz markers
    target_marker_publisher = this->create_publisher<MarkerArray>("/target_pose_rviz_markers", 10);
    virtual_marker_publisher = this->create_publisher<MarkerArray>("/virtual_pose_rviz_markers", 10);

    // Subscribe to the target and virtual poses
    target_pose_sub = this->create_subscription<Pose>(
      "px100_target_pose", 10, std::bind(&AdmittanceRvizMarkers::target_pose_callback, this, _1)
    );
    virtual_pose_sub = this->create_subscription<Pose>(
      "px100_virtual_pose", 10, std::bind(&AdmittanceRvizMarkers::virtual_pose_callback, this, _1)
    );
  }

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr target_marker_publisher;
  rclcpp::Publisher<MarkerArray>::SharedPtr virtual_marker_publisher;
  rclcpp::Subscription<Pose>::SharedPtr target_pose_sub;
  rclcpp::Subscription<Pose>::SharedPtr virtual_pose_sub;

  // Assign unique marker IDs for each pose type (target and virtual)
  const int TARGET_POSE_ID = 0;
  const int VIRTUAL_POSE_ID = 1;

  void target_pose_callback(const Pose::SharedPtr msg) {
    // First, clear the old markers for the target pose
    clear_old_markers(target_marker_publisher);

    // Create a marker array and add both pose and text markers for the target_pose
    MarkerArray marker_array;
    marker_array.markers.push_back(create_pose_marker(*msg, "target_pose", TARGET_POSE_ID, 1.0, 0.0, 0.0)); // Red for target pose
    marker_array.markers.push_back(create_text_marker(*msg, "target_pose", TARGET_POSE_ID));

    // Publish the marker array on the target marker topic
    target_marker_publisher->publish(marker_array);
  }

  void virtual_pose_callback(const Pose::SharedPtr msg) {
    // First, clear the old markers for the virtual pose
    clear_old_markers(virtual_marker_publisher);

    // Create a marker array and add both pose and text markers for the virtual_pose
    MarkerArray marker_array;
    marker_array.markers.push_back(create_pose_marker(*msg, "virtual_pose", VIRTUAL_POSE_ID, 0.0, 1.0, 0.0)); // Green for virtual pose
    marker_array.markers.push_back(create_text_marker(*msg, "virtual_pose", VIRTUAL_POSE_ID));

    // Publish the marker array on the virtual marker topic
    virtual_marker_publisher->publish(marker_array);
  }

  Marker create_pose_marker(const Pose &pose, const std::string &label, int marker_id, float r, float g, float b) {
    // Create the pose marker (as a smaller arrow)
    Marker marker;
    marker.header.frame_id = "world";  // Set frame of reference
    marker.header.stamp = this->now();
    marker.ns = label;
    marker.id = marker_id;  // Use consistent marker IDs to overwrite old markers
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.pose = pose;

    // Even smaller scale for the arrow
    marker.scale.x = 0.1;  // Shorter arrow length
    marker.scale.y = 0.01; // Narrower arrow width
    marker.scale.z = 0.01; // Shorter arrow height

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;  // Fully opaque
    return marker;
  }

  Marker create_text_marker(const Pose &pose, const std::string &label, int marker_id) {
    // Create the text marker
    Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = label;
    marker.id = marker_id + 100;  // Ensure text markers have different IDs from pose markers
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.action = Marker::ADD;

    // Position the text slightly above the pose
    marker.pose.position = pose.position;
    marker.pose.position.z += 0.05;  // Text slightly raised

    // Even smaller scale for the text
    marker.scale.z = 0.02;  // Smaller text size

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;  // Fully opaque
    marker.text = label;  // Set the text label ("target_pose" or "virtual_pose")
    return marker;
  }

  void clear_old_markers(rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher) {
    // Create a marker array with a DELETEALL action to clear previous markers
    MarkerArray marker_array;
    Marker delete_marker;
    delete_marker.action = Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // Publish the marker array to clear old markers
    marker_publisher->publish(marker_array);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdmittanceRvizMarkers>());
  rclcpp::shutdown();
  return 0;
}
