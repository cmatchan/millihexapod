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
        millihex = Millihexapod()
        rate = rospy.Rate(0.5)
        
        millihex.down()
        rate.sleep()

        millihex.compute_ik()

        # millihex.up(joint_angle = -(np.pi /2))
        # rate.sleep()

        # millihex.down()
        # rate.sleep()
        
        # joints = np.array([0, 6, 12])
        # target_joint_position = np.pi / 4
        # millihex.set_joint_positions(joints, target_joint_position)
        # rate.sleep()
        
        # millihex.down()
        # rate.sleep()

        sys.exit(0)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())