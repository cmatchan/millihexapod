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
        rospy.init_node('robot_walk', anonymous=True)       # Initialize ROS node
        rate = rospy.Rate(1)

        # Initialize Millihexapod object
        millihex = Millihexapod()
        
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

        # # Test joint movement at 100 Hz
        # x = 0.0
        # angle_rate = rospy.Rate(100)
        # while (x < np.pi):
        #     for i in range((int) (millihex.num_legs / 2)):
        #         i = i + 1
        #         angle = np.sin(x) * (np.pi / 2)
        #         print(f"angle = {angle}")

        #         millihex.set_joint_position(i, 1, -angle)
        #         millihex.set_joint_position(i + 3, 1, angle)

        #         millihex.set_joint_position(i, 2, -angle)
        #         millihex.set_joint_position(i + 3, 2, angle)

        #         millihex.set_joint_position(i, 3, -angle)
        #         millihex.set_joint_position(i + 3, 3, angle)

        #         angle_rate.sleep()
        #         x = x + 0.005

        # rate.sleep()

        # # Test millihex tripod gait
        # while not rospy.is_shutdown():
        #     millihex.tripod_gait()
        #     print(f"stance_state: {millihex.get_stance_state()}")
        #     print(f"swing_state: {millihex.get_swing_state()}\n")
        #     rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())