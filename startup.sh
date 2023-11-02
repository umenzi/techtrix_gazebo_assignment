#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch techtrix_gazebo techtrix.launch &
roslaunch techtrix_gazebo techtrix_rviz.launch &
roslaunch techtrix_control techtrix_control.launch &
