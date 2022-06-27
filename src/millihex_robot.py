#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Robot:
    def __init__(self, num_legs, joints_per_leg):
        """Initialize ROS publishers & ROS subscribers."""
        self.num_legs = num_legs
        self.joints_per_leg = joints_per_leg
        self.num_joint_publishers = 0       # Number of joint publishers started

        def start_joint_position_controller_publishers(self):
            """Initialize joint publishers for the topic
            /millihex/leg#_joint#_position_controller/command."""
            for i in range(self.num_legs):
                for j in range(self.joints_per_leg):
                    leg_number = i + 1        # Robot leg indexes from 1
                    joint_number = j + 1      # Robot joint indexes from 1

                    # Name of joint publisher = "leg#_joint#_pub"
                    publisher_name = f"leg{leg_number}_joint{joint_number}_pub"

                    # Name of topic = "/millihex/leg#_joint#_position_controller/command"
                    publisher_topic = f"/millihex/leg{leg_number}" \
                        f"_joint{joint_number}_position_controller/command"

                    # Initialize publisher command
                    start_publisher_command = f"self.{publisher_name} = " \
                        f"rospy.Publisher('{publisher_topic}', Float64, queue_size=10, latch=True)"

                    # Execute command string as a function
                    exec(start_publisher_command)

                    check_num_pub_connections = f"self.{publisher_name}.get_num_connections() < 1"
                    while exec(check_num_pub_connections):
                        rate = rospy.Rate(1)
                        rate.sleep()

                    self.num_joint_publishers = self.num_joint_publishers + 1

        # Initialize all joint position controllers for command topic
        start_joint_position_controller_publishers(self)

        # Subscribe to millihex/joint_states topic
        self.subscriber = rospy.Subscriber("/millihex/joint_states", \
            JointState, self.callback)

        # Robot joint positions
        # angles = [-pi/2, pi/2]
        self.joint_positions = tuple()          

        # Leg stance state = {-1, 0, 1}
        # Lying down = -1
        # Low stance = 0
        # High stance = 1
        self.stance_state = (-1) + np.zeros(self.num_legs)

        # Leg swing state = {0, 1}
        # Neutral = 0
        # Forward swing = 1
        self.swing_state = np.zeros(self.num_legs)      


    def callback(self, ros_data):
        """Subscriber callback function of /millihex/joint_states topic."""
        self.joint_positions = ros_data.position    # Update joint positions
        rate = rospy.Rate(1)
        rate.sleep()


    def get_joint_index(self, leg_number, joint_number):
        """Computes the index of a given leg joint for a joint vector."""
        joint_index = (leg_number - 1) * self.joints_per_leg + (joint_number - 1)
        return joint_index


    def get_joint_position(self, leg_number, joint_number):
        """Returns the current position of joint_number in leg_number."""
        joint_index = self.get_joint_index(leg_number, joint_number)
        return self.joint_positions[joint_index]


    def get_all_joint_positions(self):
        """Returns a tuple of all robot joint positions."""
        return self.joint_positions


    def set_joint_position(self, leg_number, joint_number, joint_position):
        """Publishes joint_position to specified leg_number, joint_number."""
        # Name of joint publisher = "leg#_joint#_pub"
        publisher_name = f"leg{leg_number}_joint{joint_number}_pub"

        # Publish joint_position to joint publisher
        publish_command = f"self.{publisher_name}.publish({joint_position})"

        exec(publish_command)       # Execute string as a function


    def get_stance_state(self):
        """Returns stance state vector of robot joints."""
        return self.stance_state


    def get_swing_state(self):
        """Returns swing state vector of robot joints."""
        return self.swing_state


    def set_leg_stance(self, legs = [], stance = -1):
        """Commands leg stance position.
        Lay down: stance = -1
        Low stance: stance = 0
        High stance: stance = 1"""
        for leg_number in legs:
            if stance == 0:         # Set joint angle for low stance
                joint_angle = 0.2
            elif stance == 1:       # Set joint angle for high stance
                joint_angle = 0.6
            else:                   # Stance = -1, set leg to lie down
                joint_angle = 0.0
                self.set_joint_position(leg_number, 2, joint_angle)

            self.stance_state[leg_number - 1] = stance

            # LHS leg joints have flipped axes
            if leg_number > (int) (self.num_legs / 2):
                joint_angle = (-1) * joint_angle

            # Use joints 1 and 3 for stance positioning
            self.set_joint_position(leg_number, 1, joint_angle)
            self.set_joint_position(leg_number, 3, joint_angle)

    
    def set_leg_swing(self, legs = [], swing = 0):
        """Commands leg swing position.
        Neutral: swing = 0
        Swing forward: swing = 1"""
        for leg_number in legs:
            if swing == 1:          # Set joint angle to forward swing
                joint_angle = 0.6
            else:                   # Swing = 0, set to neutral swing
                joint_angle = 0.0

            self.swing_state[leg_number - 1] = swing

            # LHS leg joints have flipped axes
            if leg_number > (int) (self.num_legs / 2):
                joint_angle = (-1) * joint_angle

            # Use joint 2 for swing positioning
            self.set_joint_position(leg_number, 2, joint_angle)


    def stand_up(self):
        """Commands robot to stand up with all legs in low stance position."""
        legs = np.arange(0,self.num_legs) + 1
        self.set_leg_stance(legs, stance = 0)
    

    def lay_down(self):
        """Commands robot to lay down flat."""
        legs = np.arange(0,self.num_legs) + 1
        self.set_leg_stance(legs, stance = -1)
        

    def tripod_gait(self):
        """Initializes and plans Tripod gait leg movement.
        Tripod leg state pattern is the following:
                        Leg indices = [1 2 3 4 5 6]
        Dominant Right stance state = [1 0 1 0 1 0]
         Dominant Left stance state = [0 1 0 1 0 1] """
        # Legs numbers for Dominant Right/Left stances
        dominant_right_legs = np.arange(0, self.num_legs, 2) + 1
        dominant_left_legs = np.arange(1, self.num_legs, 2) + 1

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