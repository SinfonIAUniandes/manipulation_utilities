#!/usr/bin/env python3

# Standard library imports
import csv

# ROS and third-party imports
import rospy

# Local application/library specific imports
import ConsoleFormatter

# ROS messages and services
from manipulation_msgs_pytoolkit.srv import *
from std_srvs.srv import SetBool, SetBoolRequest

class ManipulationPytoolkit:
    
    # ===================================================== INIT ==================================================================

    def __init__(self):

        rospy.init_node('ManipulationPytoolkit', anonymous=True)
        
        # ==============================  MANIPULATION SERVICES DECLARATION ========================================
        
        print(consoleFormatter.format('waiting for go_to_State service!', 'WARNING'))
        self.goToState= rospy.Service("manipulation_utilities/go_to_state", go_to_state, self.callback_go_to_state)
        print(consoleFormatter.format('go_to_state on!', 'OKGREEN'))

        print(consoleFormatter.format('waiting for go_to_action service!', 'WARNING'))  
        self.goToAction = rospy.Service("manipulation_utilities/go_to_action", go_to_action, self.callback_go_to_action)
        print(consoleFormatter.format('go_to_action on!', 'OKGREEN'))

        print(consoleFormatter.format('waiting for grasp_object service!', 'WARNING'))
        self.graspObject = rospy.Service("manipulation_utilities/grasp_object", grasp_object, self.callback_grasp_object)
        print(consoleFormatter.format('graspObjectPytoolkit on!', 'OKGREEN'))
        
        print(consoleFormatter.format('waiting for moveHead service!', 'WARNING'))
        self.moveHead = rospy.Service("manipulation_utilities/move_head", move_head, self.callback_move_head)
        print(consoleFormatter.format('moveHeadPytoolkit on!', 'OKGREEN'))

        
        # =============================== PYTOOLKIT SERVICES DECLARATION ========================================
        
        print(consoleFormatter.format('waiting for goToStatePytoolkit service!', 'WARNING'))  
        self.setState = rospy.ServiceProxy("manipulation_utilities/goToState", go_to_state)
        print(consoleFormatter.format('goToStatePytoolkit connected!', 'OKGREEN'))

        print(consoleFormatter.format('waiting for set_angle_srv from pytoolkit!', 'WARNING'))  
        self.motionSetAngleClient = rospy.ServiceProxy("pytoolkit/ALMotion/set_angle_srv", set_angle_srv)
        print(consoleFormatter.format('motionSetAngleServer connected!', 'OKGREEN'))  

        # Autonomous life 
        print(consoleFormatter.format('waiting for set_state_srv from pytoolkit!', 'WARNING'))  
        self.motionSetStatesClient = rospy.ServiceProxy("pytoolkit/ALAutonomousLife/set_state_srv", SetBool)
        print(consoleFormatter.format('set_state_srv connected!', 'OKGREEN')) 

        # Stiffness in pepper
        print(consoleFormatter.format('waiting for set_stiffness_srv from pytoolkit!', 'WARNING'))  
        self.motionSetStiffnessesClient = rospy.ServiceProxy("pytoolkit/ALMotion/set_stiffnesses_srv", set_stiffnesses_srv)
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
        
        # Define the list of arm joints to set stiffness for
        self.joints = ["LHand", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
                       "RHand","RShoulderPitch", "RShoulderRoll", "RElbowYaw","RElbowRoll", "RWristYaw"] 

        # Set maximum stiffness for each specified joint
        for joint in self.joints: 
            req_stiffnesses.names = joint
            req_stiffnesses.stiffnesses = 1
            self.motionSetStiffnessesClient.call(req_stiffnesses)
        
########################################  MANIPULATION SERVICES  ############################################

    # ================================== GO TO STATE ========================================

    def callback_go_to_state(self,req):
        """
        Executes a specific pose for the pepper based on the provided request.

        This callback function receives a request containing the name of the desired pose or movement and the velocity
        at which the movement should be performed. It retrieves the corresponding angles for the pose from a predefined
        CSV file containing pose information. Then, it sets the pepper joints to these angles at the specified velocity.

        Parameters:
            req (RequestMessageType): A request object containing the following attributes:
                - name (str): The name of the pose or movement to execute. This name should correspond to an entry
                in the 'objects_poses.csv' file.
                - velocity (float): The speed at which the joints should move to reach the specified angles.

        Returns:
            ResultType: The result of the service call to set the pepper joint angles. The specific type of this
            result depends on the service definition used by `motionSetAngleClient`.

        Raises:
            Exception: If there's an error in reading the CSV file or if the requested pose is not found in the file.
        """

        # Joints categories
        request = set_angle_srvRequest()
        name = req.name
        velocity = req.velocity
        joints_arms = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        joints_left_arm = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        joints_right_arm = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        joints_head = ["HeadPitch", "HeadYaw"]
        joint_left_hand = ["LHand"]
        joint_right_hand = ["RHand"]
        joint_hands = ["LHand", "RHand"]        
        
        # Read pose angles from CSV file located in data
        poses_info = csv.DictReader(open('../data/objects_poses.csv', encoding="utf-8"),delimiter=",")
        poses_angles = {key: [row[key].strip() for row in poses_info if row[key].strip()] for key in poses_info.fieldnames}
        angle = poses_angles[name]
        request.name = poses_angles[name][-1]

        request.angle = angle
        request.speed = velocity
        res = self.motionSetAngleClient.call(request)
        return res.result

    # ================================== GO TO ACTION ========================================

    def callback_go_to_action(self, req):
        request = set_angle_srvRequest()
        name = req.name
        angle = []

        joints_arms_hands = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LHand", "RHand"]
        joints_left_arm = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        joints_right_arm = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]    
        joints_arms = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        joints_hip = ["HipPitch"]
        joint_hands = ["LHand", "RHand"]
        joint_left_hand = ["LHand"]
        joint_right_hand = ["RHand"]

        if(name=="place_both_arms"):
            # Bajar cadera
            angle_1 = [-0.39]

            # Abre las manos
            angle_2 = [1.0, 1.0]

            # Separa los brazos
            angle_3 = [0.00383162, 3.8160, -0.00378999, -0.00877543, -4.09046e-05, -9.91789e-05, -3.8160, -9.95662e-05, 0.00881285, 7.63354e-05, 0.5, 0.5]

            # giras los brazos
            angle_4 = [1.55179, 1.56201, 0.0039061, -0.00881861, 0.00328999, 1.57469, -1.56204, 1.21692e-05, 0.00882497, -9.82796e-05, 0-5, 0.5]

            # Bajas los brazos
            angle_5 = [1.57471, 0.0940139, 0.00371943, -0.00879696, 0.00338298, 1.57466, -0.0797469, 2.59231e-05, 0.0087464, 3.54903e-05, 0-5, 0.5]
            
            # Subir cadera
            angle_6 = [-0.1]

            request.name = joints_hip
            request.angle = angle_1
            request.speed = 0.2
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.name = joint_hands
            request.angle = angle_2
            request.speed = 0.2
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.name = joints_arms_hands
            request.angle = angle_3
            request.speed = 0.1
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.angle = angle_4
            request.speed = 0.25
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(0.5)

            request.angle = angle_5
            request.speed = 0.15
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.name = joints_hip
            request.angle = angle_6
            request.speed = 0.15
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            return "place both arms executed"
        
        elif(name=="place_left_arm"):
            # Giras brazo
            angle_1 = [0.522444, 0.00885305, -1.39163, -0.517197, 1.57039]

            # Abrir mano
            angle_3 = [1.0]

            # Girar brazo
            angle_4 = [0.796941, 1.56198, -1.39166, -0.00880171, 1.3969]

            # Girar brazo
            angle_5 = [1.56715, 1.56184, -1.39167, -0.00898501, 1.39683]

            # Bajar brazo
            angle_6 = [1.56708, 0.0230135, -1.39941, -0.00876288, -0.230052]

            request.name = joints_left_arm
            request.angle = angle_1
            request.speed = 0.2
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.angle = angle_2
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.name = joint_left_hand
            request.angle = angle_3
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(1)

            request.name = joints_left_arm
            request.angle = angle_4
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.angle = angle_5
            request.speed = 0.2
            res = self.motionSetAngleClient.call(request)

            request.angle = angle_6
            request.speed = 0.1
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(1.5)

            return "place left arm executed"

        elif(name=="place_right_arm"):
            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ["HipPitch"]
            jointsRequest.angle = [-0.3]
            jointsRequest.speed = 0.1
            rospy.sleep(2)
            self.motionSetAngleClient.call(jointsRequest)
            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ["RWristYaw"]
            jointsRequest.angle = [-1.6]
            jointsRequest.speed = 0.2
            self.motionSetAngleClient.call(jointsRequest)
            rospy.sleep(2.2)

            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ["RHand"]
            jointsRequest.angle = [1.0]
            jointsRequest.speed = 0.1
            self.motionSetAngleClient.call(jointsRequest)


            rospy.sleep(1.2)
            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            jointsRequest.angle = [0.00385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05]
            jointsRequest.speed = 0.1
            self.motionSetAngleClient.call(jointsRequest)
            rospy.sleep(2)
            jointsRequest.angle = [1.60385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05]
            jointsRequest.speed = 0.25
            self.motionSetAngleClient.call(jointsRequest)
            rospy.sleep(0.5)
            jointsRequest.angle = [1.75007, -0.111032, 1.6967, 0.102538, -0.0100479]
            jointsRequest.speed = 0.15
            self.motionSetAngleClient.call(jointsRequest)
            rospy.sleep(2)
            jointsRequest = set_angle_srvRequest()
            jointsRequest.name = ["HipPitch"]
            jointsRequest.angle = [-0.1]
            jointsRequest.speed = 0.1
            rospy.sleep(2)
            self.motionSetAngleClient.call(jointsRequest)
            return "place right arm executed"

        elif(name=="place_right_cereal"):
            # Ajusta el brazo
            angle_1 = [0.101243, -0.0705631, 1.39132, 0.199418, 1.79627]

            request.name = joints_right_arm
            request.angle = angle_1
            request.speed = 0.1
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            # brazo para la tableta rapido
            angle_2 = [0.101191, -0.0706058, 0.430828, 1.56205, 1.79615]

            request.name = joints_right_arm
            request.angle = angle_2
            request.speed = 0.32
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(0.45)

            # Devuelve brazo a la pose inicial
            angle_3 = [0.101074, -0.0706477, 0.430851, 0.00882103, 1.79614]

            request.name = joints_right_arm
            request.angle = angle_3
            request.speed = 0.32
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(0.45)

            # Mueve al lado derecho 
            angle_4 = [0.10472, -0.0523599, 1.39626, 0.191986, 1.81514]

            request.name = joints_right_arm
            request.angle = angle_4
            request.speed = 0.08
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)
            
            return "place right cereal executed"
            
        elif(name == "request_help_both_arms"):
            angle_1 = [0.285927, 0.088337, -0.79688, -0.866391, -1.82384, 0.285973, -0.0939549, 0.789178, 0.886188, 1.82378]
            request.name = joints_arms
            request.angle = angle_1
            request.speed = 0.1
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(3)

            angle2 = [0.316383, 0.00873112, -0.690055, -0.786829, -1.82382, 0.308927, -0.00874898, 0.690114, 0.792578, 1.82378]
            request.angle = angle2
            request.speed = 0.1
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            return "request_help_both_arms"

        elif(name == "spin_head"):
            angle_1 = [-0.436332]
            request.name = ["HeadYaw"]
            request.angle = angle_1
            request.speed = 0.1
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(1)

            angle_2 = [0.436332]
            request.name = ["HeadYaw"]
            request.angle = angle_2
            request.speed = 0.08
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(3)

            request_1 = GoToStateRequest()
            request_1.name = "default_head"
            request_1.velocity = 0.1
            res = self.setState.call(request_1)

            return "spin_head"

        else: 
            return "No action name recognized"

    # ================================== GO TO STATE ========================================

    def callback_grasp_object(self, req):
        """
        Handles a grasping request for various objects by categorizing them and assigning a specific pre-defined state 
        for the pepper to execute, based on the object type.

        This callback function examines the requested object's name and categorizes the object into one of several lists.
        Depending on the category, the pepper is instructed to transition to a specific state optimized for grasping that 
        category of object. The function supports a variety of common household and kitchen items, each associated with 
        different grasping strategies.

        Parameters:
            req (RequestMessageType): A request object that includes an 'object' attribute, which is a string representing 
            the name of the object to be grasped.

        Returns:
            str or ResultType: The result of the service call to set the pepper state for grasping the object. If the object 
            name is not recognized, it returns an error message as a string. 'ResultType' should be replaced with the actual 
            type of the result provided by the 'setState' service call.

        Supported Objects:
            - List 1: Small and manageable objects like 'fork', 'spoon', 'knife', etc., are handled using the 'small_object_left_hand' state.
            - List 2: Slightly larger but flat objects like 'bowl' and 'plate' are handled using the 'bowl' state.
            - List 3: Specific items like 'mustard' are handled using a specialized 'master' state for unique cases.
        """
        request = GoToStateRequest()

        # Predefined lists categorizing objects based on the appropriate grasping strategy
        list_1 = ["fork", "spoon", "knife", "mug", "bottle", "cereal_box", "milk", "tuna",
                  "tomato_soup", "banana", "strawberry_jello", "canned_meat", "sugar"]
        list_2 = ["bowl", "plate" ]
        list_3 = ["mustard"]
        name_object = req.object 
        
        # Determine the grasping strategy based on the object category
        if(name_object in list_1):
            request.name = "small_object_left_hand"
            request.velocity = 0.1
            res = self.setState.call(request)
            return res.result
        
        elif(name_object in list_2):
            request.name = "bowl"
            request.velocity = 0.1
            res = self.setState.call(request)
            return res.result
        
        elif(name_object in list_3):
            request.name = "master"
            request.velocity = 0.1
            res = self.setState.call(request)
            return res.result

        else:
            return "Error"
        
        
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
            request.speed = 0.1
            
            # Calls the movement service and returns the result
            res = self.motionSetAngleClient.call(request)
            return res.result    
    
# =========================================================== MAIN ================================================================
        
if __name__ == '__main__':
    # Initialize a ConsoleFormatter instance for formatting console output messages.
    consoleFormatter=ConsoleFormatter.ConsoleFormatter()
    # Initialize the main class responsible for manipulation utilities in the context of this ROS node.
    manipulationPytoolkit = ManipulationPytoolkit()
    try:
        # Print a formatted success message indicating the manipulation utilities node has been successfully initialized.
        print(consoleFormatter.format(" --- manipulation utilities node successfully initialized ---","OKGREEN"))
        # Keep the node running until it's shut down, for example by a ROS shutdown signal like Ctrl+C in the terminal.
        rospy.spin()

    except rospy.ROSInterruptException:
        # Handle the case where the ROS node is interrupted (e.g., by Ctrl+C or other shutdown commands).
        # The pass statement is used here to gracefully exit the block without doing anything specific in response to the exception.
        pass