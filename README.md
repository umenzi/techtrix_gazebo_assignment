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

# General Structure

The repository consists of three different ROS packages:

- `techtrix_gazebo` contains the different models and worlds of our simulation.
  - The environment is loaded in `techtrix.world`, in the `worlds` folder.
  - The models, such as the factory, cylinder, or the conveyor belt, can be found in the `models` folder.
  - The meshes of the factory, truck, and conveyor belt were done in Blender. The original `.blend` file and the models used can be found in the `blender_environment` folder.
- `techtrix_description` contains the model and basic functionality of the TechTrix robot.
  For example, the robot model can be found in the `techtrix_robot` folder.
- `techtrix_control` contains the ROS code which controls what happens in the simulation.

# Sensors

The robot makes use of multiple sensors:

- **Thermal sensor**: The robot uses a thermal sensor to detect if humans are too close to the conveyor belt.
    If a human is detected, the robot will shut down for safety reasons.
- TODO

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