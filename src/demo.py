#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from millihex_robot import Millihexapod

def main():
    try:
        millihex = Millihexapod()
        rate = rospy.Rate(0.5)

        # millihex.down()
        # rate.sleep()

        # millihex.up()
        # rate.sleep()

        target_joint_state = millihex.compute_ik()
        millihex.set_joint_state(target_joint_state, step_rate=100, angle_step=0.01)

        sys.exit(0)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()