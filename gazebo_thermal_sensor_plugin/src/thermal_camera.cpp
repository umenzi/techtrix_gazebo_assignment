#include <thermal_sensor_plugin.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/image_encodings.h>

#include <gazebo/gazebo_config.h>

namespace gazebo
{

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    template <class Base>
    GazeboRosThermalSensor_<Base>::GazeboRosThermalSensor_()
    {
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    template <class Base>
    GazeboRosThermalSensor_<Base>::~GazeboRosThermalSensor_()
    {
    }

    template <class Base>
    void GazeboRosThermalSensor_<Base>::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        Base::Load(_parent, _sdf);
        // copying from CameraPlugin into GazeboRosCameraUtils
        this->parentSensor_ = this->parentSensor;
        this->width_ = this->width;
        this->height_ = this->height;
        this->depth_ = this->depth;
        this->format_ = this->format;

        this->image_connect_count_ = boost::shared_ptr<int>(new int);
        *this->image_connect_count_ = 0;
        this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
        this->was_active_ = boost::shared_ptr<bool>(new bool);
        *this->was_active_ = false;

        LoadImpl(_parent, _sdf);
        GazeboRosCameraUtils::Load(_parent, _sdf);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Update the controller
    template <class Base>
    void GazeboRosThermalSensor_<Base>::OnNewFrame(const unsigned char *_image,
                                                   unsigned int _width, unsigned int _height, unsigned int _depth,
                                                   const std::string &_format)
    {
        if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
            return;

        this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();

        if (!this->parentSensor->IsActive())
        {
            if ((*this->image_connect_count_) > 0)
                // do this first so there's chance for sensor to run 1 frame after activate
                this->parentSensor->SetActive(true);
        }
        else
        {
            if ((*this->image_connect_count_) > 0)
            {
                common::Time cur_time = this->world_->SimTime();
                if (cur_time - this->last_update_time_ >= this->update_period_)
                {
                    this->PutCameraData(_image);
                    this->PublishCameraInfo();
                    this->last_update_time_ = cur_time;
                }
            }
        }
    }

    template <class Base>
    void GazeboRosThermalSensor_<Base>::OnNewImageFrame(const unsigned char *_image,
                                                        unsigned int _width, unsigned int _height, unsigned int _depth,
                                                        const std::string &_format)
    {
        OnNewFrame(_image, _width, _height, _depth, _format);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Put camera_ data to the interface
    template <class Base>
    void GazeboRosThermalSensor_<Base>::PutCameraData(const unsigned char *_src, common::Time &last_update_time)
    {
        this->sensor_update_time_ = last_update_time;
        this->PutCameraData(_src);
    }

    template <class Base>
    void GazeboRosThermalSensor_<Base>::PutCameraData(const unsigned char *_src)
    {
        if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0)
            return;

        this->lock_.lock();

        // copy data into image
        this->image_msg_.header.frame_id = this->frame_name_;
        this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
        this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

        /// don't bother if there are no subscribers
        if ((*this->image_connect_count_) > 0)
        {
            this->image_msg_.width = this->width_;
            this->image_msg_.height = this->height_;
            this->image_msg_.encoding = sensor_msgs::image_encodings::MONO8;
            this->image_msg_.step = this->image_msg_.width;

            size_t size = this->width_ * this->height_;

            std::vector<uint8_t> &data(this->image_msg_.data);
            data.resize(size);

            size_t img_index = 0;

            for (size_t i = 0; i < size; ++i)
            {
                if ((_src[img_index] > 254) && (_src[img_index + 1] < 1) && (_src[img_index + 2] < 1))
                {
                    // RGB [255,0,0] translates to white (white-hot)
                    data[i] = 255;
                }
                else
                {
                    // Everything else is written to the MONO8 output image much darker
                    data[i] = (_src[img_index] + _src[img_index + 1] + _src[img_index + 2]) / 8;
                }
                img_index += 3;
            }

            // publish to ros
            this->image_pub_.publish(this->image_msg_);
        }

        this->lock_.unlock();
    }
}