#!/usr/bin/env python

from calendar import c
import rospy
import rospkg
import time

from gazebo_msgs.srv import SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel # For deleting models from the environment

from geometry_msgs.msg import Pose, Point, Twist

def spawn_cylinder(time_in_between=0, amount_of_cylinders=6):
    cylinders = []
    
    # Value of the last cylinder
    y_end = -10
    # COmpute where we start
    y_start = y_end + amount_of_cylinders
    
    rospack = rospkg.RosPack()
    spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    ros_package_location = rospack.get_path('techtrix_gazebo')
    cylinder_location = f"{ros_package_location}/models/techtrix_cylinder/model.sdf"
    
    for i in range(amount_of_cylinders):
        cylinder = {"name": f"Techtrix cylinder{i + 1}", "pose": Pose(position=Point(0, y_start, 2))}
        spawn_sdf_model(
            model_name=cylinder['name'],
            model_xml=open(cylinder_location, 'r').read(),
            robot_namespace='/techtrix_description',
            initial_pose=cylinder['pose'],
            reference_frame='world'
        ) # Adds to Gazebo
        
        cylinders.append(cylinder)
        y_start -= 1
        time.sleep(time_in_between)
    
    return cylinders

def move_cylinder(cylinders):
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    while cylinders[0]['pose'].position.y < 2.5:
        for cylinder in cylinders:
            pose = cylinder['pose']
            pose.position.y += 0.03
            
            state = ModelState()
            state.model_name = str(cylinder['name'])
            state.pose = pose
            state.reference_frame = 'world'

            set_model_state(state)
            
    time.sleep(1)
    
    # After moving the cylinders to the end of the conveyor, we move them to the start position of the simulation
    
    positions = [(0.551, 5.8106, 1.7250), (0.339, 5.8106, 1.725),
                 (0.127, 5.8106, 1.725), (-0.127, 5.8106, 1.725),
                 (-0.339, 5.8106, 1.725), (-0.551, 5.8106, 1.725)]
    
    for i, cylinder in enumerate(cylinders):
        state = ModelState()
        state.model_name = str(cylinder['name'])
        
        pose = Pose()
        pose.position.x = positions[i][0]
        pose.position.y = positions[i][1]
        pose.position.z = positions[i][2]
        state.pose = pose
        
        state.reference_frame = 'world'

        set_model_state(state)
        
    time.sleep(2)
    
    
def delete_cylinders(cylinders, time_in_between=0):
    # delete_model : gazebo_msgs/DeleteModel
    del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # Handle to model spawner
    
    for cylinder in cylinders:
        name = str(cylinder['name'])
        del_model_prox(name) # Remove from Gazebo
        time.sleep(time_in_between)

if __name__ == '__main__':
    rospy.init_node('spawn_and_move_cylinders')
    
    if not rospy.is_shutdown():
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        
        cylinders = spawn_cylinder(0.5, 6)
        
        move_cylinder(cylinders)
        
        time.sleep(2)
        
        delete_cylinders(cylinders, 0.5)