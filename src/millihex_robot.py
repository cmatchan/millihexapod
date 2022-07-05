#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# Millihex constants
NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_JOINTS = NUM_LEGS * JOINTS_PER_LEG

class Millihexapod:
    def __init__(self):
        """
        Initialize Millihex state parameters and variables.
        Start ROS publishers & ROS subscribers.
        """
        print("\nInitializing Millihexapod...\n")
        pause = rospy.Rate(2)
        pause.sleep()

        # Millihex leg and joint count
        self.num_legs = NUM_LEGS
        self.joints_per_leg = JOINTS_PER_LEG
        self.num_joints = NUM_JOINTS

        # Array of joint positions
        # angle limits = [-pi/2, pi/2] rad
        self.joint_positions = np.zeros(NUM_JOINTS)
        
        # List of joint publishers
        self.publishers = [None] * NUM_JOINTS

        # Start all joint position controllers for /command topic
        print("Waiting for Joint Position Controller Publishers...")
        self.start_joint_position_controller_publishers()
        pause.sleep()

        # Subscribe to /millihex/joint_states topic
        print("\nWaiting for Joint States Subscriber...")
        self.subscriber = rospy.Subscriber("/millihex/joint_states", \
            JointState, self.joint_states_subscriber_callback)

        # Make sure subscriber is connected before continuing
        while (self.subscriber.get_num_connections() < 1):
            continue
        
        print(f"{self.subscriber.name}, " \
              f"connections = {self.subscriber.get_num_connections()}\n")
        pause.sleep()

        print("MILLIHEXAPOD INITIALIZED\n")
        pause.sleep()


    def get_joint_index(self, leg_number, joint_number):
        """
        Returns an array of updated 
        """
        joint_index = (leg_number - 1) * JOINTS_PER_LEG + (joint_number - 1)
        return joint_index


    def start_joint_position_controller_publishers(self):
        """
        Initialize joint publishers to the topic:
            /millihex/leg#_joint#_position_controller/command
        """
        for i in range(NUM_LEGS):
            for j in range(JOINTS_PER_LEG):
                # Index leg and joint from 1
                leg_number = i + 1
                joint_number = j + 1

                # Name of topic = "/millihex/leg#_joint#_position_controller/command"
                publisher_topic = f"/millihex/leg{leg_number}" \
                    f"_joint{joint_number}_position_controller/command"

                # Initialize new publisher and add to array of publishers
                joint_index = self.get_joint_index(leg_number, joint_number)
                start_publisher_command = f"self.publishers[joint_index] = " \
                    f"rospy.Publisher('{publisher_topic}', Float64, queue_size=10, latch=True)"
                exec(start_publisher_command)

                # Make sure all publishers are connected before continuing
                while (self.publishers[joint_index].get_num_connections() < 1):
                    continue

                print(f"{self.publishers[joint_index].name}, " \
                    f"connections = {self.publishers[joint_index].get_num_connections()}")


    def joint_states_subscriber_callback(self, ros_data):
        """
        Subscriber callback function of /millihex/joint_states topic.
        """
        # Convert /joint_states joint position tuple to array
        self.joint_positions = np.asarray(ros_data.position)


    def set_joint_positions(self, joints = [], target_joint_position = 0, \
        move_rate = 100, step = 0.005):
        """
        Publishes joint_positions to corresponding joint.
        joint_positions is an array of length NUM_JOINTS.
        """
        # Set rotation speed of joint rotation
        rate = rospy.Rate(move_rate)

        # Compute average of current joint angles
        joints = np.asarray(joints)
        current_joint_positions = self.joint_positions[joints]
        current_joint_positions_avg = np.sum(np.absolute(current_joint_positions)) / joints.size
        print(f"joint_positions:\n{current_joint_positions}")
        print(f"current_joint_positions_avg: {current_joint_positions_avg}")
        print(f"target_joint_position: {target_joint_position}\n")

        # Publish incremental joint angle command to joints
        theta_right = current_joint_positions_avg
        theta_left = current_joint_positions_avg
        while (np.absolute((np.absolute(theta_right) - np.absolute(target_joint_position))) > step):
            if (target_joint_position > current_joint_positions_avg):
                theta_right -= step
                theta_left += step
            elif (target_joint_position < current_joint_positions_avg):
                theta_right += step
                theta_left -= step

            middle_joint = int(NUM_JOINTS / 2)
            for j in joints[joints < middle_joint]:
                self.publishers[j].publish(theta_right)
            for j in joints[joints >= middle_joint]:
                self.publishers[j].publish(theta_left)

            rate.sleep()


    def up(self, joint_angle = 0.4, move_rate = 100, step = 0.005):
        """
        Commands robot to stand up with all legs in low stance position.
        """
        joints = np.arange(NUM_JOINTS)
        self.set_joint_positions(joints, joint_angle, move_rate, step)
    

    def down(self, move_rate = 100, step = 0.005):
        """
        Commands robot to lay down flat.
        """
        joints = np.arange(NUM_JOINTS)
        self.set_joint_positions(joints, 0.0, move_rate, step)
        

    def compute_ik(self):
        pass