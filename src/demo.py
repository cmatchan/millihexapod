#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from millihex_robot import Millihexapod

def main():
    try:
        millihex = Millihexapod()

        # millihex.walk(pattern="bipod", h=(np.pi/4), w=(np.pi/4))
        millihex.walk(pattern="tripod", h=(np.pi/3), w=(np.pi/3))

        # millihex.random_dancing()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()