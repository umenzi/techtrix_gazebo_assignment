# Gazebo Assignment

This repository contains the code to run the simulation of the TechTrix robot, {name}, for the Gazebo Assignment.

## General Structure

The `gazeboModels` folder contain a lot of models, mainly from [here](https://github.com/osrf/gazebo_models).
The models made by TechTrix are named `techtrix_X` or `TechTrix_X`, where `X` is some string.
For example, the robot model can be found in the `techtrix_robot` folder.

The structure and content of each folder is explained [here](https://classic.gazebosim.org/tutorials?tut=build_robot&cat=build_robot)
and [here](https://classic.gazebosim.org/tutorials?tut=attach_meshes&cat=build_robot).

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

## ROS

Note that this is a ROS workspace (TODO), hence:
1. Run `catkin init` to see whether the ROS workspace is correctly setup.
2. To build the workspace again, run `catkin clean -y && catkin build`.
3. To run the URDF factory, use the command `roslaunch techtrix_gazebo visualize_hrwros.launch`.
   - If the package cannot be found, your workspace is not correctly set up.