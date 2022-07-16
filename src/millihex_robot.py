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

        # Set sleep rate to pause between messages
        pause = rospy.Rate(2)
        pause.sleep()

        # Millihex leg and joint count
        self.num_legs = NUM_LEGS
        self.joints_per_leg = JOINTS_PER_LEG
        self.num_joints = NUM_JOINTS

        # Array of joint positions
        # Angle limits = [-pi/2, pi/2] rad
        # Joint array order:
        #   [leg1_joint1 leg1_joint2 leg1_joint3
        #    leg2_joint1 leg2_joint2 leg2_joint3
        #    leg3_joint1 leg3_joint2 leg3_joint3
        #    leg4_joint1 leg4_joint2 leg4_joint3
        #    leg5_joint1 leg5_joint2 leg5_joint3
        #    leg6_joint1 leg6_joint2 leg6_joint3]
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
        # Leg and joint number indices start from 1
        for leg_number in range(1, NUM_LEGS + 1):
            for joint_number in range(1, JOINTS_PER_LEG + 1):
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


    def set_joint_positions(self, joints=[], target_joint_position=0.0, \
        step_rate=100, step=0.01):
        """
        Publishes joint_positions to corresponding joint.
        joint_positions is an array of length NUM_JOINTS.
        """
        # Set rotation speed of joint rotation
        rate = rospy.Rate(step_rate)

        # Get current positions of joints to rotate
        current_joint_positions = self.joint_positions[joints]

        # Right side joints rotate upwards by Right-Hand-Rule
        # Left side joints rotate downwards by Right-Hand-Rule
        rotation_directions = np.ones(np.size(joints))
        middle_joint = int(NUM_JOINTS / 2)
        rotation_directions[joints < middle_joint] *= -1

        # Desired joint position target angle
        target_joint_positions = np.zeros(np.size(joints)) + target_joint_position
        target_joint_positions *= rotation_directions

        # Incremental joint angles
        theta = current_joint_positions
        d_theta = target_joint_positions - theta

        while (np.any(np.abs(d_theta) >= step)):
            # Compute angle step
            theta_step = np.sign(d_theta) * step
            theta += theta_step
            d_theta = target_joint_positions - theta
            
            # Publish incremental joint angles
            for i in range(np.size(joints)):
                self.publishers[joints[i]].publish(theta[i])
            rate.sleep()


    def up(self, joint_angle=0.4, step_rate=100, step=0.01):
        """
        Commands robot to stand up with all legs in low stance position.
        """
        joints = np.arange(NUM_JOINTS)
        self.set_joint_positions(joints, joint_angle, step_rate, step)
    

    def down(self, step_rate=100, step=0.01):
        """
        Commands robot to lay down flat.
        """
        joints = np.arange(NUM_JOINTS)
        self.set_joint_positions(joints, 0.0, step_rate, step)
        

    def compute_ik(self):
        pass