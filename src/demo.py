#!/usr/bin/env python3

import time
import rospy
import subprocess
import numpy as np
from millihex_robot import Millihexapod

def main():
    """
    Usage: $ rosrun millihexapod demo.py
    """

    try:
        # Initialize Millihex robot
        millihex = Millihexapod()

        # Start roscore
        subprocess.Popen('roscore')
        time.sleep(1)

        # Initialize rospy node
        rospy.init_node('robot_rock', anonymous=True)
        rospy.sleep(1)

        # Start Gazebo
        millihex.spawn_model("gazebo")

        # Gait and obstacle parameters
        x = np.pi/3
        z = np.pi/3
        stance = np.pi/4
        step = 0.02
        h = 0.05

        # Data collection loop
        while True:
            # Spawn models
            millihex.spawn_model("obstacle", args=[f"obstacle_h:={h}"])
            millihex.spawn_model("millihex")

            # Start walking test
            millihex.walk(pattern="tripod", gait_x=x, gait_z=z, stance=stance, step=step)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()