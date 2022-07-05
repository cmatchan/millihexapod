#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from millihex_robot import Millihexapod

def main():
    """
    Initializes ROS node for robot.
    """
    try:
        # Initialize ROS node and Millihexapod
        rospy.init_node('robot_walk', anonymous=True)
        millihex = Millihexapod()
        rate = rospy.Rate(1)
        
        # Command Millihex to lie down
        print("MILLIHEX DOWN:")
        millihex.down()
        rate.sleep()

        # Command Millihex to stand up
        print("MILLIHEX UP:")
        millihex.up()
        rate.sleep()

        # Command Millihex to lie down
        print("MILLIHEX DOWN:")
        millihex.down()
        rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())