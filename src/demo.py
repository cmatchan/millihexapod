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
        rospy.sleep(1)

        millihex.delete_model("millihex")
        rospy.sleep(1)

        millihex.delete_model("obstacle")
        rospy.sleep(1)

        # obstacle_args = ["obstacle_h:=0.3"]
        # millihex.execute_launch_file(millihex.spawn_obstacle_launch_path, obstacle_args)

        millihex.spawn_model("millihex")

        # # Start Millihex robot walking tests
        # millihex.walk(pattern="tripod", h=(np.pi/3), w=(np.pi/3), stance=(np.pi/4))

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()