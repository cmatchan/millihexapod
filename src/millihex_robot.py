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
        pause = rospy.Rate(1)
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
        pause.sleep()
        pause.sleep()
        self.start_joint_position_controller_publishers()
        pause.sleep()

        # Subscribe to /millihex/joint_states topic
        print("\nWaiting for Joint States Subscriber...")
        self.subscriber = rospy.Subscriber("/millihex/joint_states", JointState, self.callback)

        # Make sure subscriber is connected before continuing
        while (self.subscriber.get_num_connections() < 1):
            pause.sleep()
        
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
                    rate = rospy.Rate(100)
                    rate.sleep()

                print(f"{self.publishers[joint_index].name}, " \
                    f"connections = {self.publishers[joint_index].get_num_connections()}")


    def callback(self, ros_data):
        """
        Subscriber callback function of /millihex/joint_states topic.
        """
        # Convert /joint_states joint position tuple to array
        self.joint_positions = np.asarray(ros_data.position)
        rate = rospy.Rate(100)
        rate.sleep()

    
    def init_rotations_array(self):
        """
        Returns a joint position array following joint axis polarity
        for right side and left side legs according to Right-Hand-Rule.
        
        Positive joint effort causes right side links to rotate up.
        Positive joint effor causes left side links to rotate down.

        Returns:
            [-1 -1 -1 -1 -1 -1 -1 -1 -1  1  1  1  1  1  1  1  1  1]
        """
        # Get index of middle joint
        middle_joint = int(NUM_JOINTS / 2)

        # Set right legs to move in opposite direction to left legs
        position_array = np.ones(NUM_JOINTS)
        position_array[:middle_joint] = -1

        return position_array


    def set_joint_positions(self, joints = [], target_joint_position = 0, move_rate = 100):
        """
        Publishes joint_positions to corresponding joint.

        joint_positions is an array of length NUM_JOINTS.
        """
        # Set rotation speed of joint rotation
        rate = rospy.Rate(move_rate)

        # Compute average of current joint angles
        joints = np.asarray(joints)
        current_joint_positions = self.joint_positions[joints]
        current_joint_positions_avg = np.sum(current_joint_positions) / current_joint_positions.size
        print(f"current_joint_positions_avg: {current_joint_positions_avg}")
        print(f"target_joint_position: {target_joint_position}\n")

        # theta = current_joint_positions_avg
        # if (target_joint_position > current_joint_positions_avg & ):
        #     theta 

        for j in joints:
            self.publishers[j].publish(target_joint_position)

        rate.sleep()


    def up(self, joint_angle = 0.4, move_rate = 100):
        """
        Commands robot to stand up with all legs in low stance position.
        """
        # Set joint position array
        new_joint_positions = self.init_rotations_array()
        new_joint_positions *= joint_angle

        self.set_joint_positions(target_joint_positions = new_joint_positions, move_rate = move_rate)
    

    def down(self, move_rate = 100):
        """
        Commands robot to lay down flat.
        """

        joints = np.arange(NUM_JOINTS)
        self.set_joint_positions(joints)
        

    def compute_ik(self):
        return