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
        rate = rospy.Rate(0.5)
        
        millihex.down()
        rate.sleep()

        # millihex.up(joint_angle = -(np.pi /2))
        # rate.sleep()

        # millihex.down()
        # rate.sleep()
        
        joints = np.array([1, 4, 13])
        target_joint_position = -np.pi / 4
        millihex.set_joint_positions(joints, target_joint_position)
        rate.sleep()

        sys.exit(0)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())