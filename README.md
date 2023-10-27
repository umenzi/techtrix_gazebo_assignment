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
