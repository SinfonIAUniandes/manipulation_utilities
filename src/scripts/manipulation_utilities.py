#!/usr/bin/env python3

# Standard library imports
import csv

# ROS and third-party imports
import rospy
import rosservice

# Local application/library specific imports
import ConsoleFormatter

# ROS messages and services
from std_srvs.srv import SetBool, SetBoolRequest
from robot_toolkit_msgs.srv import set_angle_srv, set_angle_srvRequest
from manipulation_msgs.srv import *

class ManipulationPytoolkit:
    
    # ===================================================== INIT ==================================================================

    def __init__(self):

        rospy.init_node('ManipulationPyServices', anonymous=True)
        
        # =====================================  JOTINS DECLARATION ================================================
        
        # Define the lists of global joints
        self.joints = ["LHand", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
                       "RHand","RShoulderPitch", "RShoulderRoll", "RElbowYaw","RElbowRoll", "RWristYaw",
                       "HeadPitch", "HeadYaw"]
        self.joints_arms = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
                            "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        self.joints_left_arm = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        self.joints_right_arm = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        self.joints_head = ["HeadPitch", "HeadYaw"]
        self.joint_left_hand = ["LHand"]
        self.joint_right_hand = ["RHand"]
        self.joint_hands = ["LHand", "RHand"]
        self.joints_hip = ["HipPitch"]
        
        self.speed = 0.1 # Default movement speed
                
        # ==============================  MANIPULATION SERVICES DECLARATION ========================================
        
        print(consoleFormatter.format('waiting for go_to_pose service!', 'WARNING'))
        self.go_to_pose= rospy.Service("manipulation_utilities/go_to_pose", go_to_pose, self.callback_go_to_pose)
        self.set_state = rospy.ServiceProxy("manipulation_utilities/go_to_pose", go_to_pose)
        print(consoleFormatter.format('Service go_to_pose from manipulation_services is on!', 'OKGREEN'))

        print(consoleFormatter.format('waiting for go_to_sequence service!', 'WARNING'))
        self.go_to_sequence = rospy.Service("manipulation_utilities/go_to_sequence", go_to_sequence, self.callback_go_to_sequence)
        print(consoleFormatter.format('Service go_to_sequence from manipulation_services is on!', 'OKGREEN'))    

        print(consoleFormatter.format('waiting for move_head service!', 'WARNING'))
        self.move_head = rospy.Service("manipulation_utilities/move_head", move_head, self.callback_move_head)
        print(consoleFormatter.format('Service move_head from manipulation_services is on!', 'OKGREEN'))    
        
        # ==================================  MOTION SERVICES DECLARATION ======================================== 
        
        print(consoleFormatter.format('waiting for move_head service!', 'WARNING'))
        self.motion_set_angle_client = rospy.ServiceProxy("pytoolkit/ALMotion/set_angle_srv", set_angle_srv)
        print(consoleFormatter.format('set_angle_srv connected!', 'OKGREEN'))  
        
        # Autonomous life 
        print(consoleFormatter.format('waiting for set_state_srv from pytoolkit!', 'WARNING'))  
        self.motion_set_states_client = rospy.ServiceProxy("pytoolkit/ALAutonomousLife/set_state_srv", SetBool)
        print(consoleFormatter.format('set_state_srv connected!', 'OKGREEN')) 


        # Stiffness in pepper
        print(consoleFormatter.format('waiting for set_stiffness_srv from pytoolkit!', 'WARNING'))  
        self.motion_set_stiffnesses_client = rospy.ServiceProxy("pytoolkit/ALMotion/set_stiffnesses_srv", set_stiffnesses_srv)
        print(consoleFormatter.format('set_stiffness_srv connected!', 'OKGREEN')) 
        
        self.initialize()
        
    # =============================== INITIALIZE ========================================
    
    def initialize(self):
        """
        Initializes the pepper by turning off autonomous life features and enabling stiffness in its joints.

        This function performs two main operations to prepare the pepper for tasks. First, it disables the pepper
        autonomous life features to ensure it remains stationary and does not perform any autonomous actions during
        the initialization process. Second, it enables stiffness in all specified joints of the pepper, particularly
        in the arms, to prepare them for movement and interaction.

        Attributes:
            joints (list of str): A list of joint names for the pepper arms, including hands, shoulders, elbows,
            and wrists. This list is used to specify which joints will have their stiffness settings adjusted.

        Operations:
            1. Disables the pepper autonomous life features to prevent it from performing any autonomous actions
            during the initialization.
            2. Iterates over the specified arm joints and enables maximum stiffness in each to prepare them for
            controlled movements.
        """

        # Turn off autonomous life 
        req_states = SetBoolRequest()
        req_states.data = False

        # Prepare the request for setting joint stiffnesses
        req_stiffnesses = set_stiffnesses_srvRequest()

        # Set maximum stiffness for each specified joint
        for joint in self.joints: 
            req_stiffnesses.names = joint
            req_stiffnesses.stiffnesses = 1
            self.motion_set_stiffnesses_client(req_stiffnesses)
        
        print(consoleFormatter.format('Initialization of the robot Pepper Completed!', 'OKGREEN'))
        
########################################  MANIPULATION SERVICES  ############################################

    # ================================== GO TO POSE ========================================

    def callback_go_to_pose(self, req):
        """
        Executes a specific pose for the pepper based on the provided request.

        This callback function receives a request containing the name of the desired pose or movement and the speed
        at which the movement should be performed. It retrieves the corresponding angles for the pose from a predefined
        CSV file containing pose information. Then, it sets the pepper joints to these angles at the specified speed.

        Parameters:
            req (RequestMessageType): A request object containing the following attributes:
                - name (str): The name of the pose or movement to execute. This name should correspond to an entry
                in the 'objects_poses.csv' file.
                - speed (float): The speed at which the joints should move to reach the specified angles.

        Returns:
            ResultType: The result of the service call to set the pepper joint angles. The specific type of this
            result depends on the service definition used by `motion_set_angle_client`.

        Raises:
            Exception: If there's an error in reading the CSV file or if the requested pose is not found in the file.
        """

        # Joints categories
        request = set_angle_srvRequest()
        name = req.name

        poses_info = csv.reader(open('src/manipulation_utilities/src/data/objects_poses.csv', 'r', encoding='utf-8'))
        
        poses_angles = {}
        for row in poses_info:
            pose_name = row[0]
            angles = []
            for value in row[1:-1]:
                try:
                    angles.append(float(value.strip()))
                except ValueError:
                    pass
            joint_category = row[-1].strip()
            poses_angles[pose_name] = (angles, joint_category)

        if name in poses_angles:
            angle, joint_category = poses_angles[name]
            if hasattr(self, joint_category):
                request = set_angle_srvRequest()
                request.name = getattr(self, joint_category)
                request.angle = angle
                request.speed = req.speed
                res = self.motion_set_angle_client.call(request)
                return res.result
        else:
            return (f"Pose '{name}' not found in the poses information.")
        
    def callback_go_to_sequence(self, req):
        """
        Executes a list of poses for the pepper based on the provided request.

        Parameters:
            req (RequestMessageType): A request object containing the following attributes:
                - name (str): The name of the pose or movement to execute. This name should correspond to an entry
                in the 'objects_poses.csv' file.
                - speed (float): The speed at which the joints should move to reach the specified angles.

        Returns:
            ResultType: The result of the service call to set the pepper joint angles. The specific type of this
            result depends on the service definition used by `motion_set_angle_client`.

        Raises:
            Exception: If there's an error in reading the CSV file or if the requested pose is not found in the file.
        """

        for i in req.poses:
            msg = self.callback_go_to_pose(self, i)
            return msg



    # ==================================== MOVE HEAD =====================================
    
    def callback_move_head(self, req):
        """
        Handles the request to move the pepper head to the specified angles in the request.

        This function is a callback used to move the pepper head to desired angles, specified
        in degrees. It converts the angles to radians and sends them to a pepper movement service. The function
        checks the angle limits for 'HeadPitch' to ensure the movement is possible.

        Parameters:
            req (RequestMessageType): A request object containing the desired angles for 'HeadPitch'
            and 'HeadYaw'. 'angle1' corresponds to 'HeadPitch' and 'angle2' to 'HeadYaw'. The angles should be specified
            in degrees.

        Returns:
            str or ResponseMessageType: If the angle for 'HeadPitch' is outside the allowed limits (-36 to 12 degrees),
            it returns an error message as a string. If the angles are valid, it sends a request to the pepper
            movement service and returns the result of that service call.

        """
        request = set_angle_srvRequest()
        joints_head = ["HeadPitch", "HeadYaw"]
        
        # Checks limits for the 'HeadPitch' angle
        if ((req.angle1 < -36) or (req.angle1 > 12)):
            return "It is not possible to move the head (HeadPitch joint) to this angle."
        
        else: 
            # Converts angles from degrees to radians
            angle1 = (req.angle1 * 3.1416) / (180)
            angle2 = (req.angle2 * 3.1416) / (180)
            
            # Sets up the request for the movement service
            request.name = joints_head
            request.angle = [angle1, angle2]
            request.speed = self.speed
            
            # Calls the movement service and returns the result
            res = self.motion_set_angle_client.call(request)
        return res.result    
    
# =========================================================== MAIN ================================================================
        
if __name__ == '__main__':
    # Initialize a ConsoleFormatter instance for formatting console output messages.
    consoleFormatter=ConsoleFormatter.ConsoleFormatter()
    try:
        # Initialize the main class responsible for manipulation utilities in the context of this ROS node.
        manipulation_py_services = ManipulationPytoolkit()
        # Print a formatted success message indicating the manipulation utilities node has been successfully initialized.
        print(consoleFormatter.format(" --- manipulation utilities node successfully initialized ---","OKGREEN"))
        # Retrieve a list of available ROS services to ensure ROS functionalities are active and accessible.
        available_services = rosservice.get_service_list()
        # Keep the node running until it's shut down, for example by a ROS shutdown signal like Ctrl+C in the terminal.
        rospy.spin()

    except rospy.ROSInterruptException:
        # Handle the interruption of the ROS node (e.g., Ctrl+C).
        pass
    
    except rospy.ROSException as e:
        # Handle general ROS errors, including the unavailability of ROS.
        print(consoleFormatter.format(f"ROS Error: {e}", "FAIL"))