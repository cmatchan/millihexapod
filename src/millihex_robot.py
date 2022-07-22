#!/usr/bin/env python3
import sys
import rospy
import numpy as np
import moveit_commander
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK

# Millihex constants
NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_JOINTS = NUM_LEGS * JOINTS_PER_LEG

class Millihexapod:
    def __init__(self):
        """
        Initializes a Millihex object and starts ROS publishers & ROS subscribers.

        Attributes
        ----------
        robot: RobotCommander
            moveit_commander robot object to get it's current state.
        num_legs: int
            

        Methods
        -------

        """
        # Initialize moveit_commander and rospy node
        print("\n==================== Initializing Millihexapod ====================")

        # Remap /joint_states to /millihex/joint_states topic for moveit
        joint_state_topic = ['joint_states:=/millihex/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        self.robot = moveit_commander.RobotCommander()
        rospy.init_node('millihex_robot', anonymous=True)

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
        print("\nWaiting for Joint Position Controller Publishers...")
        self.start_joint_position_controller_publishers()

        # Subscribe to /millihex/joint_states topic
        print("\nWaiting for Joint States Subscriber...")
        self.subscriber = rospy.Subscriber("/millihex/joint_states", \
            JointState, self.joint_states_subscriber_callback)

        # Make sure subscriber is connected before continuing
        while (self.subscriber.get_num_connections() < 1):
            continue
        
        print(f"{self.subscriber.name}, " \
              f"connections = {self.subscriber.get_num_connections()}")
        pause.sleep()

        print("==================== Millihexapod Initialized ====================\n")
        pause.sleep()


    def get_joint_index(self, leg_number, joint_number):
        """
        Returns the index of a joint in the joint_positions array.
        
        Parameters
        ----------

        Returns
        -------

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


    def set_joint_state(self, target_joint_state=[], step_rate=100, angle_step=0.01):
        """
        Publishes desired joint state to robot.
        """
        # Set rotation rate
        rate = rospy.Rate(step_rate)

        # Get current joint state
        theta = self.joint_positions

        # Compute angle distance to travel
        d_theta = np.asarray(target_joint_state) - theta

        while (np.any(np.abs(d_theta) >= angle_step)):
            # Compute the next step angle for all joints
            step = np.sign(d_theta) * angle_step
            theta += step
            d_theta -= step
            
            # Publish new joint positions
            for i in range(NUM_JOINTS):
                self.publishers[i].publish(theta[i])
            rate.sleep()


    def up(self, step_rate=100, angle_step=0.01):
        """
        Commands robot to stand up with all legs in low stance position.
        """
        print("MILLIHEX UP\n")
        target_joint_state = np.zeros(NUM_JOINTS) + (np.pi / 6)
        middle_joint = int(NUM_JOINTS / 2)
        target_joint_state[0:middle_joint] *= -1
        self.set_joint_state(target_joint_state, step_rate=100, angle_step=0.01)
    

    def down(self, step_rate=100, angle_step=0.01):
        """
        Commands robot to lay down flat.
        """
        print("MILLIHEX DOWN\n")
        target_joint_state = np.zeros(NUM_JOINTS)
        self.set_joint_state(target_joint_state, step_rate=100, angle_step=0.01)
        

    def compute_ik(self):
        """
        Commands robot to lay down flat.
        """
        print("COMPUTE IK")

        # List all the leg groups of Millihex:
        group_names = self.robot.get_group_names()
        print(f"Planning Groups: {group_names}\n")

        # Define move_group 'leg1'
        print("Initializing leg1 move_group...")
        leg1_move_group = moveit_commander.MoveGroupCommander("leg1")

        # Get move_group properties
        leg1_joints = leg1_move_group.get_joints()
        leg1_current_pose = leg1_move_group.get_current_pose()
        eef_link = leg1_move_group.get_end_effector_link()

        # Get a random pose goal
        leg1_pose_goal = leg1_move_group.get_random_pose(eef_link)
        print(f"\nTarget Pose:\n{leg1_pose_goal}\n")

        # Connect to /compute_ik service
        rospy.wait_for_service('compute_ik')
        try:
            moveit_compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        req = PositionIKRequest()
        req.group_name = 'leg1'
        req.robot_state = self.robot.get_current_state()
        req.ik_link_name = eef_link
        req.pose_stamped = leg1_pose_goal
        req.timeout = rospy.Duration(10)

        # Request IK computation from MoveIt
        resp = moveit_compute_ik(req)
        target_joint_state = resp.solution.joint_state.position
        print(f"\nTarget Joint State:\n{target_joint_state}\n")
        
        return target_joint_state

        # # Plan and visualize trajectory in RViz
        # print("COMMAND POSE")
        # leg1_move_group.set_joint_value_target(leg1_pose_goal, eef_link, True)
        # plan = leg1_move_group.go(wait=True)
        # leg1_move_group.stop()