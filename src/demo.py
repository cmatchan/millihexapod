#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from millihex_robot import Robot

def main():
    """Initializes ROS node for robot"""
    try:
        rospy.init_node('robot_walk', anonymous=True)       # Initialize ROS node
        rate = rospy.Rate(0.5)        # Set refresh rate

        # Initialize a Robot object to spawn millihex
        num_legs = 6
        joints_per_leg = 3
        millihex = Robot(num_legs, joints_per_leg)

        # Confirm that all publishers connected
        print("ROBOT INITIALIZED")
        print(f"Joint Position Publisher connections = {millihex.num_joint_publishers}\n")

        # Wait for rosnode to connect to subscriber
        print("Waiting for Joint States Subscriber...")
        while millihex.subscriber.get_num_connections() < 1:
            rate.sleep()

        print(f"Joint States Subscriber connections = {millihex.subscriber.get_num_connections()}\n")
        
        # Reset millihex to lying down
        millihex.lay_down()
        print(f"stance_state: {millihex.get_stance_state()}")
        print(f"swing_state: {millihex.get_swing_state()}\n")
        rate.sleep()

        # Test joint movement at 100 Hz
        x = 0.0
        angle_rate = rospy.Rate(100)
        while (x < np.pi):
            for i in range((int) (num_legs / 2)):
                i = i + 1
                angle = np.sin(x) * (np.pi / 2)
                print(f"angle = {angle}")

                millihex.set_joint_position(i, 1, -angle)
                millihex.set_joint_position(i + 3, 1, angle)

                millihex.set_joint_position(i, 2, -angle)
                millihex.set_joint_position(i + 3, 2, angle)

                millihex.set_joint_position(i, 3, -angle)
                millihex.set_joint_position(i + 3, 3, angle)

                angle_rate.sleep()
                x = x + 0.005

        rate.sleep()
        
        # Reset millihex to lying down
        millihex.lay_down()
        print(f"stance_state: {millihex.get_stance_state()}")
        print(f"swing_state: {millihex.get_swing_state()}\n")
        rate.sleep()
        
        # Test millihex stand up
        millihex.stand_up()
        print(f"stance_state: {millihex.get_stance_state()}")
        print(f"swing_state: {millihex.get_swing_state()}\n")
        rate.sleep()

        # Test millihex tripod gait
        while not rospy.is_shutdown():
            millihex.tripod_gait()
            print(f"stance_state: {millihex.get_stance_state()}")
            print(f"swing_state: {millihex.get_swing_state()}\n")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    sys.exit(main())