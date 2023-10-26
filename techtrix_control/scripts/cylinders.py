#!/usr/bin/env python

import rospy
import rospkg
import time

from gazebo_msgs.srv import SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel # For deleting models from the environment

from geometry_msgs.msg import Pose, Point

def spawn_cylinder(rospack, time_in_between=0):
    cylinders = []
    y_value = 0
    
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
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
        ) # Adds to Gazebo
        
        cylinders.append(cylinder)
        y_value -= 1
        time.sleep(time_in_between)
    
    return cylinders
    
def delete_cylinders(cylinders, time_in_between=0):
    # delete_model : gazebo_msgs/DeleteModel
    del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # Handle to model spawner
    
    for cylinder in cylinders:
        name = str(cylinder['name'])
        del_model_prox(name) # Remove from Gazebo
        time.sleep(time_in_between)

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
    
    if not rospy.is_shutdown():
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        
        rospack = rospkg.RosPack()
        
        cylinders = spawn_cylinder(rospack, 0.5)
        
        # for index, cylinder in enumerate(cylinders):
        #     move_cylinder(cylinder, index)
        
        delete_cylinders(cylinders, 0.5)