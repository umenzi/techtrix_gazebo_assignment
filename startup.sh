#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch techtrix_gazebo techtrix.launch &
sleep 5
roslaunch techtrix_gazebo techtrix_rviz.launch &
sleep 5
roslaunch techtrix_control techtrix_control.launch &
