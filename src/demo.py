#!/usr/bin/env python3

from re import X
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
        rospy.sleep(1)

        # Gait and obstacle parameters
        x = np.pi/3
        z = np.pi/3
        stance = np.pi/4
        step = 0.02
        h = 0.3

        # Start Millihex robot walking tests
        millihex.walk(pattern="tripod", gait_x=x, gait_z=z, stance=stance, step=step)

        # # Respawn models to restart test
        # millihex.spawn_model("obstacle", args=[f"obstacle_h:={h}"])
        # millihex.spawn_model("millihex", args=[f"obstacle_h:={h}"])


    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()