# Gazebo Assignment

This repository contains the code to run the simulation of the TechTrix robot, {name}, for the Gazebo Assignment.

## Prerequisites

Install the gazebo_ros_pkgs [here](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

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

roslaunch techtrix_gazebo techtrix.launch
roslaunch techtrix_control techtrix_control.launch
```

If you are using a Zsh terminal, you must source `setup.zsh` instead.

## ROS workspace

Note that this is a ROS workspace, hence:

1. Run `catkin init` to see whether the ROS workspace is correctly setup.
2. To build the workspace again, run `catkin clean -y && catkin build`.
3. To run the URDF factory, use the command `roslaunch techtrix_urdf_factory visualize_hrwros.launch`.
   - If the package cannot be found, your workspace is not correctly set up.

# General Structure

The repository consists of three different ROS packages:

- `techtrix_gazebo` contains the different models and worlds of our simulation.
  - The environment is loaded in `techtrix.world`, in the `worlds` folder.
  - The Techtrix robot is the `techtrix_robot` model into the `models` folder.
- `techtrix_description` contains the model and basic functionality of the Techtrix robot.
    For example, the robot model can be found in the `techtrix_robot` folder.
- `techtrix_control` contains the ROS code which controls what happens in the simulation.

## URIs

Note that we use `model://` in URIs, so you need to make sure the following:

1. Find where gazebo is: `> whereis gazebo`.
   - For example, in `/usr/share/gazebo`
2. Source the gazebo setup file: `> <gazebo_path>/gazebo/setup.sh`
   - For example, `> source /usr/share/gazebo/setup.sh`.
3. Check what are the model paths: `echo $GAZEBO_MODEL_PATH`
   - If the path to the repo's `gazeboModel` folder is printed, nice.
     You don't need to continue with the next steps.
4. We add the new model path with the following command: `> export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path_to_models_folder>`
5. Go back to step 3.