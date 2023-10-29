#include "thermal_camera.cpp"

// gazebo stuff
#include <gazebo/plugins/DepthCameraPlugin.hh>

namespace gazebo
{
    template <>
    void GazeboRosThermalSensor_<DepthCameraPlugin>::LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        this->camera_ = this->DepthCameraPlugin::depthCamera;
    }

    template class GazeboRosThermalSensor_<DepthCameraPlugin>;
    typedef GazeboRosThermalSensor_<DepthCameraPlugin> GazeboRosThermalSensor;

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(GazeboRosThermalSensor)
}