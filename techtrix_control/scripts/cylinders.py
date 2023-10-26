#!/usr/bin/env python

import rospy
import sys
import os
from gazebo_msgs.srv import SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point
import rospkg

def spawn_cylinder(rospack):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    cylinders = []
    y_value = 0
    
    ros_package_location = rospack.get_path('techtrix_gazebo')
    cylinder_location = f"{ros_package_location}/models/techtrix_cylinder/model.sdf"
    
    for i in range(6):
        cylinder = {"name": f"'Techtrix cylinder_{i}", "pose": Pose(position=Point(0, y_value, 1.45))}
        spawn_model_client(
            model_name=cylinder['name'],
            model_xml=open(cylinder_location, 'r').read(),
            robot_namespace='/techtrix_description',
            initial_pose=cylinder['pose'],
            reference_frame='world'
        )
        
        cylinders.append(cylinder)
        y_value -= 1
        rospy.sleep(0.01)
    
    return cylinders
    
# def delete_cylinder():
#     print("")
#     for index in range(6):
#         name = {model_name: 'Techtrix cylinder_{index}'}
#         os.system(f"rosservice call gazebo/delete_model {name}")
#         rospy.sleep(5)

# def move_cylinder(cylinder, index):
#     rospy.wait_for_service('/gazebo/set_model_state')
#     set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
#     cylinder.pose.position.y += 1
    
#     state = ModelState()
#     state.model_name = 'Techtrix cylinder_{index}'
#     state.pose.position.y = cylinder.pose.position.y

#     set_model_state(state)

if __name__ == '__main__':
    rospy.init_node('spawn_and_move_cylinders')
    
    rospack = rospkg.RosPack()
    
    cylinders = spawn_cylinder(rospack)
    
    # rospy.sleep(5)
    
    # for index, cylinder in enumerate(cylinders):
    #     move_cylinder(cylinder, index)
    
    # rospy.sleep(5)
    
    # delete_cylinder()

    # # Get the model path from the launch file parameter
    # model_path = rospy.get_param('cylinder_description')

    # # Define your robot names, positions, and time intervals
    # robot_info = [
    #     {"name": "robot1", "x": 0.0, "y": 0.0, "z": 0.0, "interval": 5},
    #     # Add more robots as needed
    # ]

    # for info in robot_info:
    #     spawn_robot(info["name"], info["x"], info["y"], info["z"], model_path)
    #     rospy.sleep(info["interval"])

    # # After spawning, move the robots to the right
    # for info in robot_info:
    #     move_robot(info["name"], x_offset=1.0)  # Adjust x_offset as needed
