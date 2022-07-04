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
                    self.num_joint_publishers = self.num_joint_publishers + 1

        # Initialize all joint position controllers for command topic
        start_joint_position_controller_publishers(self)

        # Subscribe to /millihex/joint_states topic
        self.subscriber = rospy.Subscriber("/millihex/joint_states", JointState, self.callback)

        # Joint positions (rad)
        # angle range = [-pi/2, pi/2]
        self.joint_positions = np.zeros(NUM_JOINTS)

        # Leg stance state = {-1, 0, 1}
        # Lying down = -1
        # Low stance = 0
        # High stance = 1
        self.stance_state = (-1) + np.zeros(NUM_LEGS)

        # Leg swing state = {0, 1}
        # Neutral = 0
        # Forward swing = 1
        self.swing_state = np.zeros(NUM_LEGS)      


    def callback(self, ros_data):
        """
        Subscriber callback function of /millihex/joint_states topic.
        """
        # Convert /joint_states joint position tuple to an array
        self.joint_positions = np.asarray(ros_data.position)
        rate = rospy.Rate(100)
        rate.sleep()


    def get_joint_index(self, leg_number, joint_number):
        """
        Computes the index of a given leg joint for a joint vector.
        """
        joint_index = (leg_number - 1) * JOINTS_PER_LEG + (joint_number - 1)
        return joint_index


    def get_joint_position(self, leg_number, joint_number):
        """
        Returns the current position of joint_number in leg_number.
        """
        joint_index = self.get_joint_index(leg_number, joint_number)
        return self.joint_positions[joint_index]


    def get_all_joint_positions(self):
        """
        Returns a tuple of all robot joint positions.
        
        joint_positions is an array of length NUM_JOINTS.
        """
        return self.joint_positions


    def set_joint_position(self, leg_number, joint_number, joint_position):
        """
        Publishes joint_position to specified leg_number, joint_number.
        """
        # Name of joint publisher = "leg#_joint#_pub"
        publisher_name = f"leg{leg_number}_joint{joint_number}_pub"

        # Publish joint_position to joint publisher
        publish_command = f"self.{publisher_name}.publish({joint_position})"
        exec(publish_command)       # Publish joint position to controller
    

    def set_joint_positions(self, new_joint_positions = np.zeros(NUM_JOINTS), move_rate = 100):
        """
        Publishes joint_positions to corresponding joint.

        joint_positions is an array of length NUM_JOINTS.
        """
        for i in range(NUM_JOINTS):
            self.publishers[i].publish(new_joint_positions[i])


    def get_stance_state(self):
        """
        Returns stance state vector of robot joints.
        """
        return self.stance_state


    def get_swing_state(self):
        """
        Returns swing state vector of robot joints.
        """
        return self.swing_state


    def set_leg_stance(self, legs = [], stance = -1):
        """
        Commands leg stance position.
        Lay down: stance = -1
        Low stance: stance = 0
        High stance: stance = 1
        """
        for leg_number in legs:
            if stance == 0:         # Set joint angle for low stance
                joint_angle = 0.3
            elif stance == 1:       # Set joint angle for high stance
                joint_angle = 0.6
            else:                   # Stance = -1, set leg to lie down
                joint_angle = 0.0
                self.set_joint_position(leg_number, 2, joint_angle)

            self.stance_state[leg_number - 1] = stance

            # LHS leg joints have flipped axes
            if leg_number > (int) (NUM_LEGS / 2):
                joint_angle = (-1) * joint_angle

            # Use joints 1 and 3 for stance positioning
            self.set_joint_position(leg_number, 1, joint_angle)
            self.set_joint_position(leg_number, 3, joint_angle)

    
    def set_leg_swing(self, legs = [], swing = 0):
        """
        Commands leg swing position.
        Neutral: swing = 0
        Swing forward: swing = 1
        """
        for leg_number in legs:
            if swing == 1:          # Set joint angle to forward swing
                joint_angle = 0.8
            else:                   # Swing = 0, set to neutral swing
                joint_angle = 0.0

            self.swing_state[leg_number - 1] = swing

            # LHS leg joints have flipped axes
            if leg_number > (int) (NUM_LEGS / 2):
                joint_angle = (-1) * joint_angle

            # Use joint 2 for swing positioning
            self.set_joint_position(leg_number, 2, joint_angle)


    def up(self):
        """
        Commands robot to stand up with all legs in low stance position.
        """
        legs = np.arange(0, NUM_LEGS) + 1
        self.set_leg_stance(legs, stance = 0)
    

    def down(self):
        """
        Commands robot to lay down flat.
        """
        legs = np.arange(0, NUM_LEGS) + 1
        self.set_leg_stance(legs, stance = -1)
        

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