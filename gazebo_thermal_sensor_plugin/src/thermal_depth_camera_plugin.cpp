#include "thermal_camera.cpp"

// gazebo stuff
#include <gazebo/plugins/DepthCameraPlugin.hh>

namespace gazebo
{
    template <>
    void GazeboRosThermalCamera_<DepthCameraPlugin>::LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        this->camera_ = this->DepthCameraPlugin::depthCamera;
    }

    template class GazeboRosThermalCamera_<DepthCameraPlugin>;
    typedef GazeboRosThermalCamera_<DepthCameraPlugin> GazeboRosThermalCamera;

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(GazeboRosThermalCamera)
}