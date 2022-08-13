#!/usr/bin/env python3

# KILLALL: $ killall -9 rosmaster & killall -9 gzserver & killall -9 gzclient

import time
import rospy
import roslaunch
import subprocess
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

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
    num_legs: int
        The number of legs in a Millihex robot. (default 6)

    joints_per_leg: int
        The number of joints in each Millihex leg. (default 3)

    num_joints: int
        The total number of joints in a Millihex robot.
        (num_legs * joints_per_leg = 18)

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
        An array of ROS Publisher objects that publish to the /command topic of
        the Millihex robot's joint position controllers.

    subscriber:
        A ROS Subscriber object that subscribes to the Millihex robot
        /joint_states topic and continuously updates the robot's current joint
        positions.

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
        Starts ROS Publishers and Subscribers for getting and setting joint
        states and properties.
        """
        
        print("\n==================== Initializing Millihexapod ====================")
        
        # Start roscore
        subprocess.Popen('roscore')
        time.sleep(1)

        # Initialize rospy node
        rospy.init_node('robot_rock', anonymous=True)
        rospy.sleep(1)
        
        # Start an empty world in Gazebo
        self.start_gazebo_launch_file_path = \
            "/root/catkin_ws/src/millihexapod/launch/start_gazebo.launch"
        self.execute_launch_file(self.start_gazebo_launch_file_path)

        # Spawn Millihex robot in Gazebo
        self.spawn_millihex_launch_path = \
            "/root/catkin_ws/src/millihexapod/launch/spawn_millihex.launch"
        self.execute_launch_file(self.spawn_millihex_launch_path)

        # Spawn Obstacle object in Gazebo
        self.spawn_obstacle_launch_path = \
            "/root/catkin_ws/src/millihexapod/launch/spawn_obstacle.launch"
        self.execute_launch_file(self.spawn_obstacle_launch_path)

        # Millihex leg and joint count
        self.num_legs = NUM_LEGS
        self.joints_per_leg = JOINTS_PER_LEG
        self.num_joints = NUM_JOINTS

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
              f"connections = {self.subscriber.get_num_connections()}\n")
        rospy.sleep(1)

        print("==================== Millihexapod Initialized =====================\n")
        rospy.sleep(1)


    def execute_launch_file(self, launch_file_path, args=[]):
        """
        Execute a launch file given it's path using the roslaunch command.

        Parameters
        ----------
        launch_file_path : str
            The full path of the launch file to execute from the /root directory.
        """

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        # Add command-line args to launch file
        cli_args = [launch_file_path]
        cli_args.extend(args)
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        # Execute launch file
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()

        # Sleep to ensure launch file is executed
        rospy.sleep(1)
        print(f"\nEXECUTED: {launch_file_path}\n")


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
                    f"rospy.Publisher('{publisher_topic}', " \
                    f"Float64, queue_size=10, latch=True)"
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


    def up(self):
        """
        Commands the Millihex robot to stand up with all legs.
        """

        print("MILLIHEX UP\n")
        target_joint_state = np.zeros(NUM_JOINTS) + (np.pi/4)
        middle_joint = int(NUM_JOINTS / 2)
        target_joint_state[0:middle_joint] *= -1
        self.set_joint_state(target_joint_state)
    
    
    def down(self):
        """
        Commands the Millihex robot to lay down flat.
        """

        print("MILLIHEX DOWN\n")
        target_joint_state = np.zeros(NUM_JOINTS)
        self.set_joint_state(target_joint_state)
    

    def stroke_control(self, stroke="down", h=(np.pi/4), w=(np.pi/4), legs=[],
            joint_state=[]):
        """
        Rotate leg joints through a specified stroke edge as part of a circular
        gait pattern.

        Parameters
        ----------
        stroke: str
            Specify Millihex leg stroke edge.
            stroke = ["down", "up", "back", "front"]

        h: float
            Sets the vertical height parameter of the leg gait (z-direction).
        
        w: float
            Sets the horizontal height parameter of the leg gait (x-direction).

        legs: int[]
            Legs for which to command joints.

        joint_state = float[]
            Current state of all joints in the Millihex robot.

        """
        
        # Check that stroke parameter is valid
        try:
            stroke in ["down", "up", "back", "front"]
        except ValueError:
            print("Invalid stroke. Must be ['down','up','back','front']")

        # Loop through leg groups to command position
        for leg in legs:
            # Get joint index to move
            leg_joint1 = self.get_joint_index(leg_number=leg, joint_number=1)
            leg_joint2 = self.get_joint_index(leg_number=leg, joint_number=2)
            leg_joint3 = self.get_joint_index(leg_number=leg, joint_number=3)

            # Define rotation orientation for right side legs
            if leg <= int(NUM_LEGS / 2):
                z = -h
                x = -w
            else:
                z = h
                x = w
                
            # Control leg stroke joint positions
            if stroke == "down":
                joint_state[leg_joint1] += z/2
                joint_state[leg_joint3] += z/2
            elif stroke == "up":
                joint_state[leg_joint1] -= z/2
                joint_state[leg_joint3] -= z/2
            elif stroke == "back":
                joint_state[leg_joint2] = 0
            else:
                joint_state[leg_joint2] += x

        # Publish joint positions
        self.set_joint_state(joint_state)
        

    def walk(self, pattern="tripod", h=(np.pi/4), w=(np.pi/4), stance=(np.pi/4)):
        """
        Commands the Millihex robot to walk with a specified gait pattern and
        parameterization.

        Parameters
        ----------
        pattern: str
            Specify Millihex leg gait pattern.
            pattern = ["bipod", "tripod"]

        h: float
            Sets the vertical height parameter of the leg gait (z-direction).
        
        w: float
            Sets the horizontal height parameter of the leg gait (x-direction).
        """

        # Bipod leg stroke pattern
        if pattern == "bipod":
            stroke_1 = [1, 6]
            stroke_2 = [2, 5]
            stroke_3 = [3, 4]
            leg_strokes = [stroke_1, stroke_2, stroke_3]

        # Tripod leg stroke pattern
        elif pattern == "tripod":
            right_stroke = [1, 3, 5]
            left_stroke = [2, 4, 6]
            leg_strokes = [right_stroke, left_stroke]

        # Initialize standing joint state array
        joint_state = np.zeros(NUM_JOINTS) + stance
        middle_joint = int(NUM_JOINTS / 2)
        joint_state[0:middle_joint] *= -1

        # Set all legs to back stroke
        self.stroke_control(h=h, w=w, legs=range(1,NUM_LEGS+1), stroke="back",
            joint_state=joint_state)

        # Loop through leg stroke groups and execute a leg stroke
        while True:
            for leg_stroke in leg_strokes:
                strokes = ["up","front","down","back"]
                for stroke in strokes:
                    self.stroke_control(stroke, h, w, legs=leg_stroke, joint_state=joint_state)