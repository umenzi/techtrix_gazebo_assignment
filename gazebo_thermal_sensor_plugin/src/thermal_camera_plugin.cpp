#include "thermal_camera.cpp"

namespace gazebo
{

    template <>
    void GazeboRosThermalSensor_<CameraPlugin>::LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        this->camera_ = this->CameraPlugin::camera;
    }

    template class GazeboRosThermalSensor_<CameraPlugin>;
    typedef GazeboRosThermalSensor_<CameraPlugin> GazeboRosThermalSensor;

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(GazeboRosThermalSensor)

}