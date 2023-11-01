#include <ignition/math/Pose3.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "techtrix_grabber_world_plugin/Toggle.h"
#include "techtrix_grabber_world_plugin/ToggleRequest.h"
#include "techtrix_grabber_world_plugin/ToggleResponse.h"
#include <boost/thread/recursive_mutex.hpp>
#include <gazebo_msgs/ContactState.h>

namespace gazebo
{
    struct FixedJoint 
    {
        std::string model1;
        physics::ModelPtr m1;
        std::string link1;
        physics::LinkPtr l1;

        std::string model2;
        physics::ModelPtr m2;
        std::string link2;
        physics::LinkPtr l2;

        physics::JointPtr joint;
    };

    // the following class is heavily based on https://github.com/pal-robotics/gazebo_ros_link_attacher
    class GrabberWorldPlugin : public WorldPlugin
    {
        private: physics::WorldPtr world;
        private: physics::PhysicsEnginePtr physics;
        private: boost::recursive_mutex* physics_mutex;

        private: ros::NodeHandle nh;
        private: ros::ServiceServer grabber_service;

        private: transport::NodePtr node;
        private: transport::SubscriberPtr contacts1_subscriber;
        private: transport::SubscriberPtr contacts2_subscriber;
        private: transport::SubscriberPtr contacts3_subscriber;
        private: transport::SubscriberPtr contacts4_subscriber;
        private: transport::SubscriberPtr contacts5_subscriber;        
        private: transport::SubscriberPtr contacts6_subscriber;
        
        private: bool is_on = false;
        private: std::vector<FixedJoint> joints;

        public: GrabberWorldPlugin() :
            nh("grabber_world_plugin_node") 
        {
        }

        public: void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
        {

            this->world = _world;
            this->physics = this->world->Physics();
            this->physics_mutex = this->physics->GetPhysicsUpdateMutex();

            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libtechtrix_grabber_world_plugin.so' in the gazebo_ros package)");
                return;
            }
            this->grabber_service = this->nh.advertiseService("grabber", &GrabberWorldPlugin::toggle_callback, this);
            
            this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->node->Init();

            // subscribe to contact sensors updates
            std::string topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/grabber_contact_sensor_sc_1_top/contacts";
            this->contacts1_subscriber = this->node->Subscribe(topic, &GrabberWorldPlugin::contacts_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/grabber_contact_sensor_sc_2_bot/contacts";
            this->contacts2_subscriber = this->node->Subscribe(topic, &GrabberWorldPlugin::contacts_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/grabber_contact_sensor_sc_3_bot/contacts";
            this->contacts3_subscriber = this->node->Subscribe(topic, &GrabberWorldPlugin::contacts_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/grabber_contact_sensor_sc_4_bot/contacts";
            this->contacts4_subscriber = this->node->Subscribe(topic, &GrabberWorldPlugin::contacts_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/grabber_contact_sensor_sc_5_bot/contacts";
            this->contacts5_subscriber = this->node->Subscribe(topic, &GrabberWorldPlugin::contacts_callback, this);
            topic = "/gazebo/default/techtrix_robot/grabbing_mechanism/grabber_contact_sensor_sc_6_bot/contacts";
            this->contacts6_subscriber = this->node->Subscribe(topic, &GrabberWorldPlugin::contacts_callback, this);
        }

        private: void contacts_callback(ConstContactsPtr& msgs)
        {
            if (!this->is_on) 
            {
                return;
            }

            for (unsigned int i = 0; i < msgs->contact_size(); i++) 
            {
                const auto& contact = msgs->contact(i);

                std::string collision1 = contact.collision1(); // in model::link::collision format
                std::string model1 = collision1.substr(0, collision1.find("::"));
                collision1.erase(0, collision1.find("::") + 2); // erase the model:: part
                std::string link1 = collision1.substr(0, collision1.find("::"));

                std::string collision2 = contact.collision2(); // in model::link::collision format
                std::string model2 = collision2.substr(0, collision2.find("::"));
                collision2.erase(0, collision2.find("::") + 2); // erase the model:: part
                std::string link2 = collision2.substr(0, collision2.find("::"));

                if (this->is_on) 
                {
                   this->attach(model1, link1, model2, link2);
                }

            }
        }

        private: bool toggle_callback(techtrix_grabber_world_plugin::Toggle::Request &req, techtrix_grabber_world_plugin::Toggle::Response &res) 
        {
            this->is_on = req.is_on;
            if (req.is_on) {
                ROS_INFO("Turning on grabbing mechanism");
            } else {
                ROS_INFO("Turning off grabbing mechanism");
                detach_all();
            }
            
            res.ok = true;
            return true;
        }

        private: bool joint_exists(std::string model1, std::string link1, std::string model2, std::string link2, FixedJoint &joint) 
        {
            for(auto it = this->joints.begin(); it != this->joints.end(); it++) 
            {
                auto isMatch = (it->model1.compare(model1) == 0) && (it->model2.compare(model2) == 0)
                        && (it->link1.compare(link1) == 0) && (it->link2.compare(link2) == 0);
                if (isMatch) 
                {
                    joint = *it;
                    return true;
                }
            }
            return false;
        }

        private: bool attach(std::string model1, std::string link1, std::string model2, std::string link2)
        {
            FixedJoint joint;
            if (joint_exists(model1, link1, model2, link2, joint))
            {
                ROS_INFO_STREAM("Joint already existed, reusing it.");
                joint.joint->Attach(joint.l1, joint.l2);
                return true;
            }
            
            ROS_INFO_STREAM("Creating a new joint.");
            
            // get models pointers
            auto base1_ptr = this->world->ModelByName(model1);
            if (base1_ptr == NULL) 
            {
                ROS_ERROR_STREAM("Model [" << model1 << "] was not found!");
                return false;
            } 

            auto base2_ptr = this->world->ModelByName(model2);
            if (base2_ptr == NULL) 
            {
                ROS_ERROR_STREAM("Model [" << model2 << "] was not found!");
                return false;
            } 

            physics::ModelPtr model1_ptr(dynamic_cast<physics::Model*>(base1_ptr.get()));
            physics::ModelPtr model2_ptr(dynamic_cast<physics::Model*>(base2_ptr.get()));

            // get links
            auto link1_ptr = model1_ptr->GetLink(link1);
            if (link1_ptr == NULL)
            {
                ROS_ERROR_STREAM("Link [" << link1 << "] was not found!");
                return false;
            }

            auto link2_ptr = model2_ptr->GetLink(link2);
            if (link2_ptr == NULL)
            {
                ROS_ERROR_STREAM("Link [" << link2 << "] was not found!");
                return false;
            }

            auto gazebo_joint = this->physics->CreateJoint("revolute", model1_ptr);

            joint.model1 = model1;
            joint.link1 = link1;
            joint.model2 = model2;
            joint.link2 = link2;
            joint.m1 = model1_ptr;
            joint.m2 = model2_ptr;
            joint.l1 = link1_ptr;
            joint.l2 = link2_ptr;
            joint.joint = gazebo_joint;
            this->joints.push_back(joint);

            // configure the joint
            gazebo_joint->Attach(link1_ptr, link2_ptr);
            gazebo_joint->Load(link1_ptr, link2_ptr, ignition::math::Pose3d());
            gazebo_joint->SetModel(model2_ptr);
            gazebo_joint->SetUpperLimit(0, 0);
            gazebo_joint->SetLowerLimit(0, 0);
            gazebo_joint->Init();
            return true;
        }

        private: bool detach_all()
        {
            for(auto it = this->joints.begin(); it != this->joints.end(); it++) 
            {
                FixedJoint& j = *it; 
                boost::recursive_mutex::scoped_lock lock(*this->physics_mutex);
                j.joint->Detach();
            }
            return true;
        }
    };

    GZ_REGISTER_WORLD_PLUGIN(GrabberWorldPlugin)
}
