#!/usr/bin/env python3

import rospy
import numpy as np
from millihex_robot import Millihexapod

def main():
    """
    Usage: $ rosrun millihexapod demo.py
    """

    try:
        # Initialize Millihex robot
        millihex = Millihexapod()

        # Start Millihex robot walking tests
        millihex.walk(pattern="tripod", h=(np.pi/3), w=(np.pi/3), stance=(np.pi/4))

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()