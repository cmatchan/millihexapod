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
        Initialize ROS publishers & ROS subscribers.
        """
        # Millihex leg and joint count
        self.num_legs = NUM_LEGS
        self.joints_per_leg = JOINTS_PER_LEG
        self.num_joints = NUM_JOINTS

        # Array of joint positions
        # angle range = [-pi/2, pi/2] rad
        self.joint_positions = np.zeros(NUM_JOINTS)
        
        # List of joint publishers
        self.publishers = [None] * NUM_JOINTS
        self.num_joint_publishers = 0

        def start_joint_position_controller_publishers(self):
            """
            Initialize joint publishers to the topic:
                /millihex/leg#_joint#_position_controller/command

            Publishers named as:
                leg#_joint#_pub
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
                    joint_index = (leg_number - 1) * JOINTS_PER_LEG + (joint_number - 1)
                    start_publisher_command = f"self.publishers[{joint_index}] = " \
                        f"rospy.Publisher('{publisher_topic}', Float64, queue_size=10, latch=True)"
                    exec(start_publisher_command)

                    # Make sure all publishers are connected before continuing
                    check_num_publisher_connections = \
                        f"self.publishers[{joint_index}].get_num_connections() < 1"
                    while exec(check_num_publisher_connections):
                        rate = rospy.Rate(100)
                        rate.sleep()
                    
                    # Increment publisher count for debugging
                    self.num_joint_publishers += 1

        # Initialize all joint position controllers for command topic
        start_joint_position_controller_publishers(self)

        # Subscribe to /millihex/joint_states topic
        self.subscriber = rospy.Subscriber("/millihex/joint_states", JointState, self.callback)


    def callback(self, ros_data):
        """
        Subscriber callback function of /millihex/joint_states topic.
        """
        # Convert /joint_states joint position tuple to array
        self.joint_positions = np.asarray(ros_data.position)
        rate = rospy.Rate(100)
        rate.sleep()

    
    def init_joint_position_array(self):
        """
        Returns a joint position array following joint axis polarity
        for right side and left side legs according to Right-Hand-Rule.
        
        Positive joint effort causes right side links to rotate up.
        Positive joint effor causes left side links to rotate down.

        Returns:
            [-1 -1 -1 -1 1 1 1 1]
        """
        # Get index of middle joint
        middle_joint = int(NUM_JOINTS / 2)

        # Set right legs to move in opposite direction to left legs
        position_array = np.ones(NUM_JOINTS)
        position_array[:middle_joint] = -1

        return position_array


    def get_joint_index(self, leg_number, joint_number):
        """
        Returns an array of updated 
        """
        joint_index = (leg_number - 1) * JOINTS_PER_LEG + (joint_number - 1)
        return joint_index
    

    def set_joint_positions(self, new_joint_positions = np.zeros(NUM_JOINTS), move_rate = 100):
        """
        Publishes joint_positions to corresponding joint.

        joint_positions is an array of length NUM_JOINTS.
        """
        # Determine rotation direction for joints
        rotation = self.init_joint_position_array()
        print(f"rotation: {rotation}\n")
        rotation[new_joint_positions < self.joint_positions] *= -1
        print(f"rotation: {rotation}\n")
        print(f"new_joint_positions: {new_joint_positions}\n")
        print(f"self.joint_positions: {self.joint_positions}\n")
        print(f"rotation: {rotation}\n")

        rate = rospy.Rate(move_rate)
        while np.any(np.absolute(self.joint_positions - new_joint_positions) > 0.1):
            for i in range(NUM_JOINTS):
                if ((self.joint_positions[i] - new_joint_positions[i]) > 0.1):
                    d_theta = self.joint_positions[i] + rotation[i] * 0.01
                    self.publishers[i].publish(d_theta)
                
            rate.sleep()

    def up(self, joint_angle = 0.4, move_rate = 100):
        """
        Commands robot to stand up with all legs in low stance position.
        """
        # Set joint position array
        new_joint_positions = self.init_joint_position_array()
        new_joint_positions *= joint_angle

        self.set_joint_positions(new_joint_positions, move_rate)
    

    def down(self, move_rate = 100):
        """
        Commands robot to lay down flat.
        """
        self.set_joint_positions(new_joint_positions = np.zeros(NUM_JOINTS), move_rate = move_rate)
        

    def compute_ik(self):
        return


    def tripod_gait(self):
        """
        Initializes and plans Tripod gait leg movement.
        Tripod leg state pattern is the following:
                        Leg indices = [1 2 3 4 5 6]
        Dominant Right stance state = [1 0 1 0 1 0]
         Dominant Left stance state = [0 1 0 1 0 1]
        """
        # Legs numbers for Dominant Right/Left stances
        dominant_right_legs = np.arange(0, NUM_LEGS, 2) + 1
        dominant_left_legs = np.arange(1, NUM_LEGS, 2) + 1

        # If not in tripod gait, then make robot stand
        # (Reset robot state to standing state)
        if np.sum(self.stance_state) < 3:
            self.stand_up()

        # If robot in standing state, then start tripod gait
        if np.sum(self.stance_state) == 0 and np.sum(self.swing_state) == 0:
            # Start Tripod gait with Dominant Right stance
            self.set_leg_stance(dominant_right_legs, stance = 1)
            return

        # If robot in Dominant Right stance, then swing Dominant Left legs
        # Otherwise, robot in Dominant Left stance, swing Dominant Right legs
        if np.sum(self.stance_state) == 3 and np.sum(self.swing_state) == 0:
            if self.stance_state[0] == 1:
                self.set_leg_swing(dominant_left_legs, swing = 1)
            else:
                self.set_leg_swing(dominant_right_legs, swing = 1)
            return

        if np.sum(self.stance_state) == 3 and np.sum(self.swing_state) == 3 \
            and self.stance_state[0] != self.swing_state[0]:
            swing_legs = np.nonzero(self.swing_state)[0] + 1
            self.set_leg_stance(swing_legs, stance = 1)
            return

        if np.sum(self.stance_state) == 6 and np.sum(self.swing_state) == 3:
            stance_legs = np.where(self.swing_state == 0)[0] + 1
            self.set_leg_stance(stance_legs, stance = 0)
            return

        if np.sum(self.stance_state) == 3 and np.sum(self.swing_state) == 3 \
            and self.stance_state[0] == self.swing_state[0]:
            swing_legs = np.nonzero(self.swing_state)[0] + 1
            self.set_leg_swing(swing_legs, swing = 0)
            return