#!/usr/bin/env python3
import sys
import rospy
from millihex_robot import Millihexapod

def main():
    try:
        millihex = Millihexapod()
        rate = rospy.Rate(0.5)

        millihex.down()
        rate.sleep()

        millihex.up()
        rate.sleep()

        millihex.random_dancing()

        # millihex.get_joint_state()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()