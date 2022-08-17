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

        # Gait and obstacle parameters, range=(min, max)
        x = np.pi/2             # (pi/8, pi/2)
        z = np.pi/2             # (pi/8, pi)
        stance = np.pi/2        # (pi/8, pi/2)
        step = 0.02             # (0.01, 0.5)
        h = 0.02                # (0.01, 0.15)
        pattern = "bipod"   # ["bipod", "tripod", "quadruped", "pentapod"]

        # Spawn models and start walk test
        millihex.spawn_model("obstacle", args=[f"obstacle_h:={h}"])
        millihex.spawn_model("millihex")
        millihex.walk(pattern=pattern, gait_x=x, gait_z=z, stance=stance, step=step)


        # Parameter sweep (N^5 data points)
        N = 4
        x_range = np.linspace(np.pi/8, np.pi/2, N)
        z_range = np.linspace(np.pi/8, np.pi, N)
        stance_range = np.linspace(np.pi/8, np.pi/2, N)
        step_range = np.linspace(0.01, 0.5, N)
        h_range = np.linspace(0.01, 0.15, N)
        pattern_range = ["bipod", "tripod", "quadruped", "pentapod"]

        # Data collection
        for x in x_range:
            for z in z_range:
                for stance in stance_range:
                    for step in step_range:
                        for h in h_range:
                            for pattern in pattern_range:
                                millihex.spawn_model("obstacle", args=[f"obstacle_h:={h}"])
                                millihex.spawn_model("millihex")
                                millihex.walk(pattern, gait_x=x, gait_z=z, stance=stance, step=step)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()