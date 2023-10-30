#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import Empty


def process_image(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

    # Check for perfect white pixels
    white_pixels = cv2.inRange(cv_image, (255, 255, 255), (255, 255, 255))

    # If there are perfect white pixels, we found an object with high temperature
    if cv2.countNonZero(white_pixels) > 10:
        rospy.loginfo("Perfect white pixels found. Resetting the world.")
        rospy.logerr("Perfect white pixels found. Resetting the world.")
        
        # Reset the world
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
            return
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    
def main():
    rospy.init_node('thermal_sensor_behaviour')
    rospy.Subscriber("/techtrix/thermal1/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__' and not rospy.is_shutdown():
        main()