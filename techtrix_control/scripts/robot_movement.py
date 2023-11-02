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
        rospy.sleep(1)
        self.base_robot(0.5)
        rospy.sleep(1)
        self.lift_robot(5)
        # rospy.sleep(1)
        time.sleep(3)

    def drop_cylinders(self):
        """
        The robot will grab the cylinders
        """
        # Drop the cylinders
        self.grabber(False)
        rospy.sleep(1)
        # Move the grabbing mechanism to the initial position
        self.lift_robot(2)
        rospy.sleep(1)

    def move(self, target_y):
        """
        We move the robot by target_y. E.g.:
            - If robot is in y=4 and target_y=2, then the resulting y is 6
            - If robot is in y=1 and target_y=-2, then the resulting y is -1
        """
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            current_y = self.get_robot_position().pose.position.y
            init_y = current_y

            # We move slowly to the desired location
            if target_y > 0:
                while current_y < target_y + init_y:
                    self.base_robot(1000)
                    
                    current_y = self.get_robot_position().pose.position.y
                    
                    # current_y += 0.01
                    # model_state.pose.position.y = current_y

                    # set_model_state(model_state)
                    # time.sleep(0.01)  # Add a small delay to make it seem fluid
            else:
                while current_y > init_y - target_y:
                    self.base_robot(-10)
                    
                    current_y = self.get_robot_position().pose.position.y
                    
                    # current_y -= 0.01
                    # model_state.pose.position.y = current_y

                    # set_model_state(model_state)
                    # time.sleep(0.01)  # Add a small delay to make it seem fluid

            # Finally, we reach the final destination
            # model_state.pose.position.y = current_y
            # set_model_state(model_state)

            return True

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

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

        # The robot then goes to the desired location with the cylinders
        executor.move(3)

        # The robot drops the cylinders
        executor.drop_cylinders()

        # We go back to the initial position
        executor.move(-3)

        # Finally, we remove the cylinders
        delete_cylinders(cylinders, 0.5)

        # Prevent this code from exiting until Ctrl+C is pressed.
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Error!")
        pass
