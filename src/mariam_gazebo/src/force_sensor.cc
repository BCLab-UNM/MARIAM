#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/ForceTorqueSensor.hh>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"

namespace gazebo
{
  class ForceSensorPlugin : public SensorPlugin {
  public:
    ForceSensorPlugin() : SensorPlugin() {}

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
      std::string topic_name = "/force";

      if (_sdf->HasElement("topic")) {
        topic_name = _sdf->Get<std::string>("topic");
      }

      // Initialize ROS2 node
      ros_node = rclcpp::Node::make_shared("force_sensor_plugin");
      force_publisher = ros_node->create_publisher<std_msgs::msg::Float32>(
        topic_name,
        10
      );

      // Ensure the sensor is a ForceTorqueSensor
      this->forceTorqueSensor = std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(_sensor);
      
      if (!this->forceTorqueSensor) {
        gzerr << "ForceSensorPlugin requires a ForceTorqueSensor.\n";
        return;
      }

      // Connect to the sensor update event
      this->updateConnection = this->forceTorqueSensor->ConnectUpdated(
          std::bind(&ForceSensorPlugin::OnUpdate, this));

      // Activate the sensor
      this->forceTorqueSensor->SetActive(true);

      std::cout << "ForceSensorPlugin loaded successfully.\n";
    }

  private:
    void OnUpdate() {
      // Get the force and torque readings
      ignition::math::Vector3d force = this->forceTorqueSensor->Force();
      // ignition::math::Vector3d torque = this->forceTorqueSensor->Torque();

      // Output the readings to the console
      // std::cout << "Force: " << force << std::endl;

      // Create a message to publish
      auto msg = std_msgs::msg::Float32();
      msg.data = force.X();
      force_publisher->publish(msg);
    }

    // Pointer to the ForceTorqueSensor
    sensors::ForceTorqueSensorPtr forceTorqueSensor;

    // Connection to the sensor update event
    event::ConnectionPtr updateConnection;

    // ros2 node and publisher
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_publisher;
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_SENSOR_PLUGIN(ForceSensorPlugin)
}