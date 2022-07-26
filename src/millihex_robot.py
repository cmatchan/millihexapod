#!/usr/bin/env python3
import sys
import rospy
import numpy as np
import moveit_commander
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK

# Millihex constants
NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_JOINTS = NUM_LEGS * JOINTS_PER_LEG

class Millihexapod:
    """
    A class used to represent and control a Millihexapod robot in Gazebo
    simulation.

    Attributes
    ----------
    robot: RobotCommander
        moveit_commander robot object to get the Millihex robot's current state.

    num_legs: int
        The number of legs in a Millihex robot. (default 6)

    joints_per_leg: int
        The number of joints in each Millihex leg. (default 3)

    num_joints: int
        The total number of joints in a Millihex robot.
        (num_legs * joints_per_leg = 18)

    group_names: str[]
        An array of the names of all the leg move groups used by MoveIt to compute
        the Inverse Kinematics. Each leg group contains link and joint relations.

        Leg group names:
            ['leg1' 'leg2' 'leg3' 'leg4' 'leg5' 'leg6' ]

    move_groups: MoveGroupCommander
        moveit_commander move_group object to get and set leg move group states
        and properties.

    joint_positions: float[]
        Stores the current state of all joints in a Millihex robot.

        Joint state array order:
            [leg1_joint1  leg1_joint2  leg1_joint3
             leg2_joint1  leg2_joint2  leg2_joint3
             leg3_joint1  leg3_joint2  leg3_joint3
             leg4_joint1  leg4_joint2  leg4_joint3
             leg5_joint1  leg5_joint2  leg5_joint3
             leg6_joint1  leg6_joint2  leg6_joint3]
    
    publishers: Publisher[]
        An array of ROS Publisher objects that publish to the /command topic of the
        Millihex robot's joint position controllers.

    subscriber:
        A ROS Subscriber object that subscribes to the Millihex robot /joint_states
        topic and continuously updates the robot's current joint positions.

    Methods
    -------
    get_joint_index(leg_number, joint_number)
        Given a desired joint and leg number, returns the index of a joint in a
        joint state array.

    start_joint_position_controller_publishers()
        Initializes all joint publishers to the joint position controller
        /command topic.

    joint_states_subscriber_callback(ros_data)
        Subscriber callback function for the /millihex/joint_states topic.

    set_joint_state(target_joint_state=[], step_rate=100, angle_step=0.01)
        Publishes the target_joint_state array of joint values to the Millihex
        joint position controllers /command topic.

    up(joint_angle=(np.pi/6), step_rate=100, angle_step=0.01)
        Commands the Millihex robot to stand up with all legs.

    down(step_rate=100, angle_step=0.01)
        Commands the Millihex robot to lay down flat.

    compute_ik()
        Computes Inverse Kinematics for a desired Millihex robot joint state.
    """

    def __init__(self):
        """
        Initializes MoveIt for leg trajectory planning.
        Starts ROS Publishers and Subscribers for getting and setting joint
        states and properties.
        """
        
        print("\n==================== Initializing Millihexapod ====================")
        
        # Remap /joint_states to /millihex/joint_states topic for MoveIt
        joint_state_topic = ['joint_states:=/millihex/joint_states']

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(joint_state_topic)
        self.robot = moveit_commander.RobotCommander()

        # Initialize rospy node
        rospy.init_node('robot_rock', anonymous=True)
        pause = rospy.Rate(2)
        pause.sleep()

        # Millihex leg and joint count
        self.num_legs = NUM_LEGS
        self.joints_per_leg = JOINTS_PER_LEG
        self.num_joints = NUM_JOINTS

        # List of leg group names
        self.group_names = self.robot.get_group_names()

        # List of all leg groups
        self.move_groups = [None] * len(self.group_names)

        # Initialize MoveGroupCommander for all leg groups
        print("\nInitializing MoveGroupCommander...")
        for i in range(len(self.group_names)):
            self.move_groups[i] = moveit_commander.MoveGroupCommander(self.group_names[i])

        # Array of joint positions
        # Angle limits = [-pi/2, pi/2] rad
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

        print("==================== Millihexapod Initialized =====================\n")
        pause.sleep()


    def get_joint_index(self, leg_number, joint_number):
        """
        Given a desired joint and leg number, returns the index of a joint in a
        joint state array.
        
        Parameters
        ----------
        leg_number : int
            The leg number of the Millihex robot (1-6).
            The leg number order is top to bottom, left to right.

        joint_number : int
            The joint number for a given leg (1-3).
            The joint number order is in to out (body to foot).

              Leg Order                    Joint Order
               (Leg #)                      [Joint #]

            (1) ----- (4)                 +----------+----+
                  |                       |         [1]   |
            (2) ----- (5)                 + [2] +----+----+
                  |                       |     |         |
            (3) ----- (6)           +-----+     |         |
                  |             (1) |    [3]    |         |
                  |                 +-----+-----+         |

        Returns
        -------
        joint_index : int
            The corresponding index for joints in a joint state array.
            The joint state array order follows the convention specified by the
            'joint_positions' attribute.
        """

        joint_index = (leg_number - 1) * JOINTS_PER_LEG + (joint_number - 1)
        return joint_index


    def start_joint_position_controller_publishers(self):
        """
        Initializes all joint publishers to the joint position controller
        /command topic. Stores initialzied publishers in the 'publishers' array
        attribute.

        Joint position controller /command topic name convention:
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
        Subscriber callback function for the /millihex/joint_states topic.
        Updates the 'joint_positions' array attribute which stores the current
        Millihex robot joint state.
        """

        # Convert /joint_states joint position tuple to array
        self.joint_positions = np.asarray(ros_data.position)


    def set_joint_state(self, target_joint_state=[], step_rate=100, angle_step=0.01):
        """
        Publishes the target_joint_state array of joint values to the Millihex
        joint position controllers /command topic.

        Parameters
        ----------
        target_joint_state : float[]
            Array of desired, target joint positions to command joint position
            controllers. The joint order follows the convention specified by the
            'joint_positions' attribute.

        setp_rate: int
            Sets the rate for publishing each incremental angle towards the target
            joint position.

        angle_step: float
            Sets the incremental angle step between each publish command sent to
            the joint position controllers.
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


    def up(self, joint_angle=(np.pi/6), step_rate=100, angle_step=0.01):
        """
        Commands the Millihex robot to stand up with all legs.

        Parameters
        ----------
        joint_angle: float
            Desired joint angle to publish to all joints.

        setp_rate: int
            Sets the rate for publishing each incremental angle towards the target
            joint position.

        angle_step: float
            Sets the incremental angle step between each publish command sent to
            the joint position controllers.
        """

        print("MILLIHEX UP\n")
        target_joint_state = np.zeros(NUM_JOINTS) + joint_angle
        middle_joint = int(NUM_JOINTS / 2)
        target_joint_state[0:middle_joint] *= -1
        self.set_joint_state(target_joint_state, step_rate=100, angle_step=0.01)
    

    def down(self, step_rate=100, angle_step=0.01):
        """
        Commands the Millihex robot to lay down flat.

        Parameters
        ----------
        setp_rate: int
            Sets the rate for publishing each incremental angle towards the target
            joint position.

        angle_step: float
            Sets the incremental angle step between each publish command sent to
            the joint position controllers.
        """

        print("MILLIHEX DOWN\n")
        target_joint_state = np.zeros(NUM_JOINTS)
        self.set_joint_state(target_joint_state, step_rate=100, angle_step=0.01)
        

    def compute_ik(self, target_leg_poses=[], eef_link_names=[]):
        """
        Computes Inverse Kinematics for a desired Millihex robot leg pose state.

        Parameters
        ----------
        target_leg_poses = PoseStamped[]
            An array of desired poses for all Millihex legs.
        """

        # Connect to /compute_ik service
        rospy.wait_for_service('compute_ik')
        try:
            moveit_compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        
        # Set parameters for IK Request
        ik_request = PositionIKRequest()
        ik_request.group_name = self.move_groups[0].get_name()
        ik_request.robot_state = self.robot.get_current_state()
        ik_request.ik_link_names = eef_link_names
        ik_request.pose_stamped_vector = target_leg_poses
        ik_request.timeout = rospy.Duration(10)

        # Get IK Response from MoveIt
        ik_response = moveit_compute_ik(ik_request)
        target_joint_state = ik_response.solution.joint_state.position
        print(f"Target Joint State:\n{target_joint_state}\n")
            
        return target_joint_state


    def random_dancing(self):
        """
        And now......Random Dancing!
        """

        print(f"And now", end="")
        pause = rospy.Rate(5)
        pause.sleep()

        for i in range(6):
            sys.stdout.write(".")
            sys.stdout.flush()
            pause.sleep()

        print(f"Random Dancing!\n")
        pause = rospy.Rate(2)
        pause.sleep()

        while True:
            target_leg_poses = [None] * NUM_LEGS
            eef_link_names = [None] * NUM_LEGS

            for i in range(NUM_LEGS):
                leg_group = self.move_groups[i + 1]
                eef_link = leg_group.get_end_effector_link()
                eef_link_names[i] = eef_link
                target_leg_poses[i] = leg_group.get_random_pose(eef_link)

            target_joint_state = self.compute_ik(target_leg_poses, eef_link_names)
            self.set_joint_state(target_joint_state, step_rate=100, angle_step=0.01)


    def triangle_gait_2d(self):
        target_leg_poses = [None] * NUM_LEGS
        eef_link_names = [None] * NUM_LEGS

        for i in range(NUM_LEGS):
            leg_group = self.move_groups[i + 1]
            eef_link = leg_group.get_end_effector_link()
            eef_link_names[i] = eef_link
            target_leg_poses[i] = leg_group.get_random_pose(eef_link)

        target_joint_state = self.compute_ik(target_leg_poses, eef_link_names)
        self.set_joint_state(target_joint_state, step_rate=100, angle_step=0.01)