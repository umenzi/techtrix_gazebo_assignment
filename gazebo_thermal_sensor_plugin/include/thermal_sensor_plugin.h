#ifndef GAZEBO_ROS_THERMAL_CAMERA_HH
#define GAZEBO_ROS_THERMAL_CAMERA_HH

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>

// other libraries for gazebo
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
    template <typename Base>
    class GazeboRosThermalCamera_ : public Base, GazeboRosCameraUtils
    {
        /// \brief Constructor
        /// \param parent The parent entity, must be a Model or a Sensor
    public:
        GazeboRosThermalCamera_();

        /// \brief Destructor
    public:
        ~GazeboRosThermalCamera_();

        /// \brief Load the plugin
        /// \param take in SDF root element
    public:
        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    public:
        void LoadImpl(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {}

        /// \brief Update the controller
    protected:
        virtual void OnNewFrame(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);

        /// \brief Update the controller
    protected:
        virtual void OnNewImageFrame(const unsigned char *_image,
                                     unsigned int _width, unsigned int _height,
                                     unsigned int _depth, const std::string &_format);

        /// \brief Put camera data to the ROS topic
    protected:
        void PutCameraData(const unsigned char *_src);

    protected:
        void PutCameraData(const unsigned char *_src, common::Time &last_update_time);
    };

}
#endif