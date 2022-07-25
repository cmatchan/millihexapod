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

        millihex.triangle_gait_2d()
        rate.sleep()

        sys.exit(0)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()