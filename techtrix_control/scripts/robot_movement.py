#!/usr/bin/env python

from cylinders import spawn_cylinder, move_cylinder, delete_cylinders
import rospy
import time
from std_msgs.msg import Float64
from techtrix_grabber_world_plugin.srv import Toggle
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState


class CommandExecutor:

    def __init__(self):
        # Initialize the ROS node here
        rospy.init_node('techtrix_robot_controller', anonymous=False)

    def lift_robot(self, val):
        """
        Lifts the grabber of the robot. Upwards if given a positive value (0 will be really slow), or
        downwards if given a negative value.
        """
        pub = rospy.Publisher('/techtrix/lifting_joint_position_controller/command', Float64, queue_size=10)
        rospy.sleep(1)  # Wait for publisher to be ready
        pub.publish(data=val)

    def base_robot(self, val):
        pub = rospy.Publisher('/techtrix/base_joint_position_controller/command', Float64, queue_size=10)
        rospy.sleep(1)  # Wait for the publisher to be ready
        pub.publish(val)

    def grabber(self, val):
        """
        Whether we grab or not.
        """
        service_proxy = rospy.ServiceProxy('/grabber_world_plugin_node/grabber', Toggle)
        service_proxy(is_on=val)

    def create_cylinders(self, time_in_between, amount_of_cylinders):
        """
        Creates several cylinders at the end of the conveyor belt, and moves them to the start of it
        (imitating how a conveyor belt works)
        """
        rospy.wait_for_service('/gazebo/spawn_sdf_model')

        new_cylinders = spawn_cylinder(time_in_between, amount_of_cylinders)
        move_cylinder(new_cylinders)

        return new_cylinders

    def grab_cylinders(self):
        """
        The robot will grab the cylinders
        """
        self.lift_robot(-1.9)
        # rospy.sleep(3)
        time.sleep(3)
        self.grabber(True)
        time.sleep(1)
        self.base_robot(0.5)
        time.sleep(1)
        self.lift_robot(5)
        # rospy.sleep(1)
        time.sleep(3)

    def drop_cylinders(self):
        """
        The robot will grab the cylinders
        """
        # self.base_robot(0.001)
        # Move the grabbing mechanism down
        self.lift_robot(-0.5)
        rospy.sleep(1)
        # Drop the cylinders
        self.grabber(False)
        rospy.sleep(1)
        # Move the grabbing mechanism to the initial position
        self.lift_robot(2)
        self.base_robot(0)
        time.sleep(2)

    def get_robot_position(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            response = get_model_state('techtrix_robot', 'world')
            return response
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return None


if __name__ == '__main__' and not rospy.is_shutdown():
    executor = CommandExecutor()

    try:
        # We start spawning and moving the cylinders
        cylinders = executor.create_cylinders(0.5, 6)

        # The robot then grabs the cylinders
        executor.grab_cylinders()
        
        time.sleep(1)
        
        # Move forwards
        
        executor.base_robot(2)
        
        time.sleep(4)

        # The robot drops the cylinders
        executor.drop_cylinders()
        
        time.sleep(3)
        
        # Move backwards
        
        executor.base_robot(-2)

        # Finally, we remove the cylinders
        delete_cylinders(cylinders, 0.2)
        
        time.sleep(1)

        # Prevent this code from exiting until Ctrl+C is pressed.
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error!")
        pass
