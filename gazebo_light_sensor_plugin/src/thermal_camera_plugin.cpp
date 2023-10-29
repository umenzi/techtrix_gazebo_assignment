#include "thermal_camera.cpp"

// gazebo stuff
#include <gazebo/plugins/CameraPlugin.hh>

namespace gazebo
{

    template <>
    void GazeboRosThermalCamera_<CameraPlugin>::LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        this->camera_ = this->CameraPlugin::camera;
    }

    template class GazeboRosThermalCamera_<CameraPlugin>;
    typedef GazeboRosThermalCamera_<CameraPlugin> GazeboRosThermalCamera;

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(GazeboRosThermalCamera)

}