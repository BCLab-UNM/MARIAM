#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/ForceTorqueSensor.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

namespace gazebo
{
  class ForceSensorPlugin : public SensorPlugin
  {
  public:
    ForceSensorPlugin() : SensorPlugin() {}

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    {
      // Ensure the sensor is a ForceTorqueSensor
      this->forceTorqueSensor = std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(_sensor);
      if (!this->forceTorqueSensor)
      {
        gzerr << "ForceSensorPlugin requires a ForceTorqueSensor.\n";
        return;
      }

      // Connect to the sensor update event
      this->updateConnection = this->forceTorqueSensor->ConnectUpdated(
          std::bind(&ForceSensorPlugin::OnUpdate, this));

      // Activate the sensor
      this->forceTorqueSensor->SetActive(true);

      gzdbg << "ForceSensorPlugin loaded successfully.\n";
    }

  private:
    void OnUpdate()
    {
      // Get the force and torque readings
      ignition::math::Vector3d force = this->forceTorqueSensor->Force();
      // ignition::math::Vector3d torque = this->forceTorqueSensor->Torque();

      // Output the readings to the console
      std::cout << "Force: " << force << std::endl;
    }

    // Pointer to the ForceTorqueSensor
    sensors::ForceTorqueSensorPtr forceTorqueSensor;

    // Connection to the sensor update event
    event::ConnectionPtr updateConnection;
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_SENSOR_PLUGIN(ForceSensorPlugin)
}