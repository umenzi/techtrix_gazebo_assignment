#include <ignition/math/Pose3.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo_msgs/ContactState.h>
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

namespace gazebo
{
    class GrabberModelPlugin : public ModelPlugin
    {
        private: physics::ModelPtr model_ptr;

        private: ros::NodeHandle nh;

        private: transport::NodePtr node;     
        private: transport::SubscriberPtr laser1_subscriber;
        private: transport::SubscriberPtr laser2_subscriber;
        private: transport::SubscriberPtr laser3_subscriber;
        private: transport::SubscriberPtr laser4_subscriber;
        private: transport::SubscriberPtr laser5_subscriber;
        private: transport::SubscriberPtr laser6_subscriber;

        private: event::ConnectionPtr update_connection;
        private: bool should_stop_robot = false;
        private: float set_point;

        private: ros::Publisher pub;
        private: ros::Subscriber sub;

        public: GrabberModelPlugin() :
            nh("grabber_world_plugin_node") 
        {
        }

        public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
        {
            this->model_ptr = _model;

            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libtechtrix_grabber_world_plugin.so' in the gazebo_ros package)");
                return;
            }
            this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->node->Init();

            this->pub = this->nh.advertise<std_msgs::Float64>("/techtrix/lifting_joint_position_controller/command", 10);
            this->sub = this->nh.subscribe("/techtrix/lifting_joint_position_controller/state", 1, &GrabberModelPlugin::state_callback, this);


            // subscribe to contact sensors updates
            std::string topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/sc_1_laser/scan";
            this->laser1_subscriber = this->node->Subscribe(topic, &GrabberModelPlugin::laser_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/sc_2_laser/scan";
            this->laser2_subscriber = this->node->Subscribe(topic, &GrabberModelPlugin::laser_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/sc_3_laser/scan";
            this->laser3_subscriber = this->node->Subscribe(topic, &GrabberModelPlugin::laser_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/sc_4_laser/scan";
            this->laser4_subscriber = this->node->Subscribe(topic, &GrabberModelPlugin::laser_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/sc_5_laser/scan";
            this->laser5_subscriber = this->node->Subscribe(topic, &GrabberModelPlugin::laser_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/sc_6_laser/scan";
            this->laser6_subscriber = this->node->Subscribe(topic, &GrabberModelPlugin::laser_callback, this);

            this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&GrabberModelPlugin::on_update, this));
        }

        private: void on_update() 
        {
            if (this->should_stop_robot) 
            {
                const auto joint = "lifting_joint";
 
                auto joint_ptr = this->model_ptr->GetJoint(joint);
                if (joint_ptr == NULL)
                {
                    ROS_ERROR_STREAM("Joint [" << joint << "] was not found!");
                    return;
                }

                // only stop the mechanism if going down
                if (this->set_point <= joint_ptr->Position()) 
                {
                    std_msgs::Float64 stop_command;
                    stop_command.data = joint_ptr->Position(0) > -1.857 ? joint_ptr->Position(0) : -1.857;

                    ROS_INFO_STREAM("Stopping the lifting mechanism");
                    pub.publish(stop_command);
                }

                this->should_stop_robot = false;
            }
        }

        private: void state_callback(const control_msgs::JointControllerState& msg) 
        {
            this->set_point = msg.set_point;
        }

        private: void laser_callback(ConstLaserScanStampedPtr& msg)
        {
            const auto& scan = msg->scan();
            if (scan.ranges_size() == 0) 
            {
                return;
            }

            // calculate the distance to an object
            auto total = 0.0f;
            for (unsigned int i = 0; i < scan.ranges_size(); i++) 
            {
                total += scan.ranges(i);
            }
            const auto distance = total / scan.ranges_size();

            // stop the grabber movement if too close to an object
            if (distance < 0.12) 
            {  
                this->should_stop_robot = true;
            }
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(GrabberModelPlugin)
}
