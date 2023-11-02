Add# Gazebo Assignment

This repository contains the code to run the simulation of the TechTrix robot, {name}, for the Gazebo Assignment.

## Prerequisites

Install the gazebo_ros_pkgs [here](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

Install ros-noetic-ros-control and ros-noetic-ros-controllers:

```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

## Installation

Run the following commands:

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/techtrix_ws/src
cd ~/techtrix_ws
catkin init
cd ~/techtrix_ws/src
# If you want to use SSH Keys:
git clone git@github.com:umenzi/techtrix_gazebo_assignment.git
# If you prefer HTTPS
# git clone https://github.com/umenzi/techtrix_gazebo_assignment.git
catkin build
source ~/techtrix_ws/src/devel/setup.bash

# Run Gazebo environment
roslaunch techtrix_gazebo techtrix.launch

# Run ROS, e.g., loads the generation of cylinders and movement of the robot
roslaunch techtrix_control techtrix_control.launch
```

If you are using a Zsh terminal, you must source `setup.zsh` instead.

## ROS workspace

Note that this is a ROS workspace, hence:

1. Run `catkin init` to see whether the ROS workspace is correctly setup.
2. To build the workspace again, run `catkin clean -y && catkin build`.
   - If you get an RLException when running the `roslaunch` commands, your workspace is not correctly set up.

## Singularity image

Run the following commands to build and run the singularity image of the project:

```bash
sudo singularity build team_4.img techtrix.def
singularity run team_4.img
```

# General Structure

The repository consists of the following ROS packages:

- `techtrix_gazebo` contains the different models and worlds of our simulation.
  - The environment is loaded in `techtrix.world`, in the `worlds` folder.
  - The models, such as the factory, cylinder, or the conveyor belt, can be found in the `models` folder.
  - The meshes of the factory, truck, and conveyor belt were done in Blender. The original `.blend` file and the models used can be found in the `blender_environment` folder.
- `techtrix_description` contains the model and basic functionality of the TechTrix robot.
  For example, the robot model can be found in the `techtrix_robot` folder.
- `techtrix_control` contains the ROS code which controls what happens in the simulation.
- `techtrix_grabber_contact_plugin` contains a plugin that visualizes the contact sensors in rviz. Creates and publishes rviz markers when there is a collision.
- `techtrix_grabber_model_plugin` contains a plugin that implements the laser sensors. Based on the distance they provide, the plugin halts the grabber's movement if the cylinders are getting too close and might collide with the grabber.
- `techtrix_grabber_world_plugin` contains a plugin that implements the contact sensors. Allows the grabber to pick up and let go of anything that makes contact/collides with the suction cups.
- `gazebo_thermal_sensor_plugin`
# Sensors

The robot makes use of multiple sensors:

- **Thermal sensor**: The robot uses a thermal sensor to detect if humans are too close to the conveyor belt.
  If a human is detected, the robot will shut down for safety reasons.
- **Contact sensors**: Each suction cup of the robot's grabber is equiped with a contact sensor. Based on these sensors the suction cups are being activated once the operator issues a command.
- **Laser sensors**: The grabber of the robot is equiped with laser sensors to prevent the grabber of hitting the cylinders while going down.

## Thermal sensor

The TechTrix robot uses a **thermal sensor** to detect when humans are too close to the conveyor belt and automatically
stops operations.

The thermal sensor is located in the `camera_link` object of the TechTrix robot URDF model.
Since there is no official implementation of a thermal sensor in Gazebo classic, a Gazebo plugin is used,
located in the `gazebo_thermal_sensor_plugin` folder.

To run the thermal sensor in the simulation:

1. Launch the Gazebo simulation: `roslaunch techtrix_gazebo techtrix.launch`.
2. Launch RViz: `roslaunch techtrix_gazebo techtrix_rviz.launch`.

- You should now see a window showing the thermal sensor's output.
- If you don't see the thermal sensor output, add a `Camera` display to RViz.
  Under `Image Topic`, set the `Camera` display to `/techtrix/thermal1/image_raw`.
- The thermal sensor output image is similar to the normal camera image (also shown in the RViz simulation),
  but everything is much darker.
- The objects of interest are shown much brighter.
  They are modeled using COLLADA and their emissive and ambient material properties are set to maximum red:
  ```xml
    <emission>
      <color sid="emission">1 0 0 1</color>
    </emission>
    <ambient>
      <color sid="ambient">1 0 0 1</color>
    </ambient>
  ```

3. Initially, no objects are shown as bright, even when spanning cylinders.
   However, there is a human in the scene, although initially they are far away from the conveyor belt:
   ![human_far_away.png](readme_images%2Fhuman_far_away.png)
4. But what happens if we move the human close to the conveyor belt? They will appear in the thermal output as a lot of bright white pixels.
   ![human_far_away.png](readme_images%2Fhuman_next_to_belt.png)
5. To start the thermal sensor, run:
   ```
   roslaunch techtrix_control techtrix_thermal.launch
   ```
   Whenever a human is found close to the conveyor belt, the Gazebo world will be reset, moving the human far away again.

## Contact sensors
The suction cups of the grabber use contact sensors to detech objects that need to be picked up.

### Visualization
  To visualize the sensors in RViz, we have created the `techtrix_grabber_contact_plugin` which publishes markers.
  The red sphere markers indicate the the specific sensor is currently making a contact.

  ![no_contacts.png](readme_images%2Fno_contacts.png)
  ![contacts.png](readme_images%2Fcontacts.png)
### Implementation
  The sensors are implemented in `techtrix_grabber_world_plugin` - a plugin that manages the suction cups - allowing them to pick up and let go of objects that make contact with the suction cups/contact sensors automatically.

  The plugin allows the grabbing to be turned on:
  ```
  rosservice call /grabber_world_plugin_node/grabber "is_on: true" 
  ```
  and off:
  ```
  rosservice call /grabber_world_plugin_node/grabber "is_on: false" 
  ```

  The idea behind the plugin is that it reads the topics provided by the sensors and if there is a collision/contact (you should see the red markers in RViz) and the grabbing is on (based on the service it advertises), then it dynamically creates a link between the grabber and the object. See the demo.

## Laser sensors
The laser sensors function as safety mechanism prevent the grabber/lifting mechanism of hitting the cylinders while going down.
### Visualization
  For visualization in RViz we are using an already existing plugin, that creates red dots where the laser meets an object.

  ![lasers1.png](readme_images%2Flasers1.png)
  ![lasers2.png](readme_images%2Flasers2.png)
### Implementation
  The sensors are implemented in `techtrix_grabber_model_plugin` (loaded on the TechTrix robot URDF model) - a plugin that issues stop commands to the lifting joint controller when neccassery. The plugin subscribes to the laser sensors and the state of the lifting joint controller. If there is an object that is sufficiantly close to the grabber and the lifting joint controller is set to lower the grabber even more, then a stop command is issued.

  Without the plugin and the sensors, the grabber collides with the cylinders:

  ![without.png](readme_images%2Fwithout.png)

  With the plugin and the input from the sensors, the grabber stops automatically before colliding:
  
  ![with.png](readme_images%2Fwith.png)