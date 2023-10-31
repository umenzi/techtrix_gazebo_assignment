#include "ContactPlugin.hh"

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
  this->nh->shutdown();

  delete this->nh;
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get the parent sensor.
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  this->x = _sdf->Get<double>("collision_x");
  this->y = _sdf->Get<double>("collision_y");
  this->z = _sdf->Get<double>("collision_z");
  this->markerPublish = _sdf->GetElement("marker_publish")->Get<std::string>();
  this->markerId = _sdf->Get<int>("marker_id"); 

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  ROS_INFO("Loading");

  this->nh = new ros::NodeHandle("/techtrix");
  this->markerPub = this->nh->advertise<visualization_msgs::Marker>(this->markerPublish, 10); 
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
    msgs::Contacts contacts;
    contacts = this->parentSensor->Contacts();

    bool hasContact = contacts.contact_size() > 0;
    if (hasContact) 
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "grabbing_mechanism";
        marker.header.stamp = ros::Time::now();
        marker.ns = "collision_markers";
        marker.id = this->markerId;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = this->x;
        marker.pose.position.y = this->y;
        marker.pose.position.z = this->z;
        marker.pose.orientation.w = 1;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0; 
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0.1);

        markerPub.publish(marker);
    }
}