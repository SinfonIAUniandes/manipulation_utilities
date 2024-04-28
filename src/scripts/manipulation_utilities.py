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

        print(consoleFormatter.format('waiting for play_action service!', 'WARNING'))  
        self.play_action = rospy.Service("manipulation_utilities/play_action", play_action, self.callback_play_action)
        print(consoleFormatter.format('Service play_action from manipulation_services is on!', 'OKGREEN'))

        print(consoleFormatter.format('waiting for grasp_object service!', 'WARNING'))
        self.grasp_object = rospy.Service("manipulation_utilities/grasp_object", grasp_object, self.callback_grasp_object)
        print(consoleFormatter.format('Service grasp_object from manipulation_services is on!', 'OKGREEN'))     
        
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

    # ================================== PLAY ACTION ========================================

    def callback_play_action(self, req):
        """
        he callback_play_action function is a method in Python that is used to control the movements of a Pepper robot.
        It takes a single parameter, req, which is a request object that contains the name of the action to be performed.
        
        The function uses the action name to determine which set of movements to execute. These movements are defined by 
        a series of angles and speeds that are sent to the motion_set_angle_client service.
        The function uses a SetAngleSrvRequest object to specify the joint names, angles, and speeds for each movement.
        
        The movements include actions such as placing both arms, placing the left or right arm, placing the right cereal,
        requesting help with both arms, and spinning the head. Each action is associated with a series of joint movements,
        each of which is executed in sequence with a delay between each movement.
        
        The function prints out a message to the console after each movement is executed, indicating the joint names and
        angles used for the movement.

        If the action name in the request object is not recognized, the function returns a string indicating that no action
        name was recognized.
        
        
        Args:
            req: Is a request object that contains the name of the action to be performed.

        Returns:
            The return type of the function is a string, which is a message indicating the result of the action.
            This could be a message indicating that a specific action was executed, or a message indicating that
            no action name was recognized.
        """
        
        
        request = set_angle_srvRequest()
        name = req.name
        
        # pose 1: Placing Pepper's both arms in the specified angles
        if(name=="place_both_arms"):
            # ===================================== Angles definition for each step ===============================================
            # 1. Baja la cadera
            hips_lower_angle = [-0.39]
            # 2. Abre las manos
            hands_parameters = [1.0, 1.0]
            # 3. Separa los brazos
            arms_separate_angles = [0.00383162, 3.8160, -0.00378999, -0.00877543, -4.09046e-05, -9.91789e-05, -3.8160, -9.95662e-05, 0.00881285, 7.63354e-05, 0.5, 0.5]
            # 4. Gira los brazos
            arms_rotation_angles = [1.55179, 1.56201, 0.0039061, -0.00881861, 0.00328999, 1.57469, -1.56204, 1.21692e-05, 0.00882497, -9.82796e-05, 0-5, 0.5]
            # 5. Bajas los brazos
            arms_lower_angles = [1.57471, 0.0940139, 0.00371943, -0.00879696, 0.00338298, 1.57466, -0.0797469, 2.59231e-05, 0.0087464, 3.54903e-05, 0-5, 0.5]
            # 6. Sube la cadera
            hips_raise_angle = [-0.1]
            
            # ===================================== Setting the angles and executing each step ===================================== 
            # 1. Baja la cadera
            request.name = self.joints_hip
            request.angle = hips_lower_angle
            request.speed = 0.2
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Hip was lowered according to the angle ', self.joints_hip, 'with the values: ', hips_lower_angle,'!', 'OKGREEN'))
            rospy.sleep(2)

            # 2. Abre las manos
            request.name = self.joint_hands
            request.angle = hands_parameters
            request.speed = 0.2
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Hands were opened according to the parameters ', self.joint_hands, ' with the values: ', hands_parameters,'!', 'OKGREEN'))
            rospy.sleep(2)

            # 3. Separa los brazos
            request.name = self.joints_arms_hands
            request.angle = arms_separate_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Arms were separated according to the angles ', self.joints_arms, ' with the values: ', arms_separate_angles,'!', 'OKGREEN'))
            rospy.sleep(2)

            # 4. Gira los brazos
            request.name = self.joints_arms_hands
            request.angle = arms_rotation_angles
            request.speed = 0.25
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Arms were rotated according to the angles ', self.joints_arms, ' with the values: ', arms_rotation_angles,'!', 'OKGREEN'))
            rospy.sleep(0.5)

            # 5. Bajas los brazos
            request.name = self.joints_arms_hands
            request.angle = arms_lower_angles
            request.speed = 0.15
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Arms were lowered according to the angles ', self.joints_arms, ' with the values: ', arms_lower_angles,'!', 'OKGREEN'))
            rospy.sleep(2)

            # 6. Sube la cadera
            request.name = self.joints_hip
            request.angle = hips_raise_angle
            request.speed = 0.15
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Hips were raised according to the angles ', self.joints_hip, ' with the values: ', hips_raise_angle,'!', 'OKGREEN'))
            rospy.sleep(2)

            return "Result: Pepper placed her both arms in the pose specified."
        
        # pose 2: Placing Pepper's left arm in the specified angles
        elif name == "place_left_arm":
            # ===================================== Angles definition for each step ===============================================
            # Primer giro del brazo
            first_left_arm_rotation_angles = [0.522444, 0.00885305, -1.39163, -0.517197, 1.57039]
            # Apertura de la mano
            left_arm_open_parameter = [1.0]
            # Segundo giro del brazo
            second_left_arm_rotation_angles = [0.796941, 1.56198, -1.39166, -0.00880171, 1.3969]
            # Tercer giro del brazo
            third_left_arm_rotation_angles = [1.56715, 1.56184, -1.39167, -0.00898501, 1.39683]
            # Desplazamiento del brazo hacia abajo
            left_arm_lower_angles = [1.56708, 0.0230135, -1.39941, -0.00876288, -0.230052]

            # ===================================== Setting the angles and executing each step ===================================== 
            # 1. First rotation of the left arm
            request.name = self.joints_left_arm
            request.angle = first_left_arm_rotation_angles
            request.speed = 0.2
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Left arm was rotated according to the angles ', self.joints_left_arm, ' with the values: ', first_left_arm_rotation_angles,'!', 'OKGREEN'))
            rospy.sleep(2)

            # 2. Open the left hand
            request.angle = left_arm_open_parameter
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Left hand was opened according to the angle ', self.joints_left_arm, ' with the values: ', left_arm_open_parameter,'!', 'OKGREEN'))
            rospy.sleep(2)

            # 3. Second rotation of the left arm
            request.name = self.joint_left_hand
            request.angle = second_left_arm_rotation_angles
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Left arm was rotated according to the angles ', self.joints_left_arm, ' with the values: ', second_left_arm_rotation_angles,'!', 'OKGREEN'))
            rospy.sleep(1)

            # 4. Third rotation of the left arm
            request.angle = third_left_arm_rotation_angles
            request.speed = 0.2
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Left arm was rotated according to the angles ', self.joints_left_arm, ' with the values: ', third_left_arm_rotation_angles,'!', 'OKGREEN'))    
            rospy.sleep(2)
            
            # 5. Lower the left arm
            request.angle = left_arm_lower_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Left arm was lowered according to the angles ', self.joints_left_arm, ' with the values: ', left_arm_lower_angles,'!', 'OKGREEN'))   
            rospy.sleep(1.5)

            return "Result: Pepper placed her left arm in the pose specified."

        # pose 3: Placing Pepper's right arm in the specified angles
        elif(name=="place_right_arm"):
            # ===================================== Angles definition for each step ===============================================
            # 1. Baja la cadera
            first_hip_pitch_angle = [-0.3]
            # 2. Gira la muneca derecha
            right_wrist_yaw_angle = [-1.6]
            # 3. Abre la mano derecha
            right_hand_open_parameter = [1.0]
            # 4. Primer giro del brazo derecho  
            first_right_shoulder_elbow_wrist_angles = [0.00385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05]
            # 5. Segundo giro del brazo derecho
            second_right_shoulder_elbow_wrist_angles = [1.60385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05]
            # 6. Tercer giro del brazo derecho
            third_right_shoulder_elbow_wrist_angles = [1.75007, -0.111032, 1.6967, 0.102538, -0.0100479]
            # 7. Sube la cadera
            second_hip_pitch_angle = [-0.1]
            
            # ===================================== Setting the angles and executing each step ===================================== 
            # 1. Baja la cadera
            joints_request = set_angle_srvRequest()
            joints_request.name = ["HipPitch"]
            joints_request.angle = first_hip_pitch_angle
            joints_request.speed = 0.1
            self.motion_set_angle_client.call(joints_request)
            print(consoleFormatter.format('Hip was lowered according to the angle ', self.joints_hip, 'with the values: ', first_hip_pitch_angle,'!', 'OKGREEN'))
            rospy.sleep(2)
            
            # 2. Gira la muneca derecha
            joints_request = set_angle_srvRequest()
            joints_request.name = ["RWristYaw"]
            joints_request.angle = right_wrist_yaw_angle
            joints_request.speed = 0.2
            self.motion_set_angle_client.call(joints_request)
            print(consoleFormatter.format('Right wrist was rotated according to the angle ', self.joints_right_arm, 'with the values: ', right_wrist_yaw_angle,'!', 'OKGREEN'))
            rospy.sleep(2.2)

            # 3. Abre la mano derecha
            joints_request = set_angle_srvRequest()
            joints_request.name = ["RHand"]
            joints_request.angle = right_hand_open_parameter   
            joints_request.speed = 0.1
            self.motion_set_angle_client.call(joints_request)
            print(consoleFormatter.format('Right hand was opened according to the parameter ', self.joint_right_hand, 'with the values: ', right_hand_open_parameter,'!', 'OKGREEN'))   
            rospy.sleep(1.2)
            
            # 4. Gira el brazo derecho
            joints_request = set_angle_srvRequest()
            joints_request.name = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
            joints_request.angle = first_right_shoulder_elbow_wrist_angles
            joints_request.speed = 0.1
            self.motion_set_angle_client.call(joints_request)
            print(consoleFormatter.format('Right arm (shoulder, elbow, wrist) was rotated for the first time according to the angles ', self.joints_right_arm, 'with the values: ', first_right_shoulder_elbow_wrist_angles,'!', 'OKGREEN'))
            rospy.sleep(2)
            
            # 5. Gira el brazo derecho
            joints_request.angle = [1.60385512, -1.56205, 0.00377709, 0.0087872, 4.07988E-05]
            joints_request.speed = 0.25
            self.motion_set_angle_client.call(joints_request)
            print(consoleFormatter.format('Right arm (shoulder, elbow, wrist) was rotated for the second time according to the angles ', self.joints_right_arm, 'with the values: ', second_right_shoulder_elbow_wrist_angles,'!', 'OKGREEN'))  
            rospy.sleep(0.5)
            
            # 6. Gira el brazo derecho
            joints_request.angle = [1.75007, -0.111032, 1.6967, 0.102538, -0.0100479]
            joints_request.speed = 0.15
            self.motion_set_angle_client.call(joints_request)
            print(consoleFormatter.format('Right arm (shoulder, elbow, wrist) was rotated for the third time according to the angles ', self.joints_right_arm, 'with the values: ', third_right_shoulder_elbow_wrist_angles,'!', 'OKGREEN'))   
            rospy.sleep(2)
            
            # 7. Sube la cadera
            joints_request = set_angle_srvRequest()
            joints_request.name = ["HipPitch"]
            joints_request.angle = [-0.1]
            joints_request.speed = 0.1
            self.motion_set_angle_client.call(joints_request)
            print(consoleFormatter.format('Hip was raised according to the angle ', self.joints_hip, 'with the values: ', second_hip_pitch_angle,'!', 'OKGREEN'))   
            rospy.sleep(2)
            
            return "Result: Pepper placed her right arm in the pose specified."

        # pose 4: Placing Pepper's right arm in the specified angles (cereal)
        elif(name=="place_right_cereal"):
            
            # ===================================== Angles definition for each step ===============================================
            # 1. Para ajustar el brazo
            arm_adjust_angles = [0.101243, -0.0705631, 1.39132, 0.199418, 1.79627]
            # 2. Para la tableta rapido
            tablet_angles = [0.101191, -0.0706058, 0.430828, 1.56205, 1.79615]
            # 3. Devuelve brazo a la pose inicial
            arm_initial_pose_angles = [0.101074, -0.0706477, 0.430851, 0.00882103, 1.79614]
            # 4. Mueve al lado derecho
            arm_move_right_angles = [0.10472, -0.0523599, 1.39626, 0.191986, 1.81514]

            # ===================================== Setting the angles and executing each step ===================================== 

            # 1. Para ajustar el brazo
            request.name = self.joints_right_arm
            request.angle = arm_adjust_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Right arm was adjusted the first time according to the angles ', self.joints_right_arm, ' with the values: ', arm_adjust_angles,'!', 'OKGREEN'))    
            rospy.sleep(2)

            # 2. Para la tableta rapido
            request.name = self.joints_right_arm
            request.angle = tablet_angles
            request.speed = 0.32
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Right arm was adjusted a second time according to the angles ', self.joints_right_arm, ' with the values: ', tablet_angles,'!', 'OKGREEN'))
            rospy.sleep(0.45)

            # 3. Devuelve brazo a la pose inicial
            request.name = self.joints_right_arm
            request.angle = arm_initial_pose_angles
            request.speed = 0.32
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Right arm was adjusted a third time according to the angles ', self.joints_right_arm, ' with the values: ', arm_initial_pose_angles,'!', 'OKGREEN'))  
            rospy.sleep(0.45)

            # 4. Mueve al lado derecho
            request.name = self.joints_right_arm
            request.angle = arm_move_right_angles
            request.speed = 0.08
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Right arm was adjusted a fourth time according to the angles ', self.joints_right_arm, ' with the values: ', arm_move_right_angles,'!', 'OKGREEN'))    
            rospy.sleep(2)
            
            return "Result: Pepper placed her right arm in the pose specified (cereal)."
            
        # pose 5: Requesting help with both arms
        elif(name == "request_help_both_arms"):
            # ===================================== Angles definition for each step ================================================
            # 1. First movement of both arms
            first_arms_help_angles = [0.285927, 0.088337, -0.79688, -0.866391, -1.82384, 0.285973, -0.0939549, 0.789178, 0.886188, 1.82378]
            # 2. Second movement of both arms
            second_arms_help_angles = [0.316383, 0.00873112, -0.690055, -0.786829, -1.82382, 0.308927, -0.00874898, 0.690114, 0.792578, 1.82378]
            
            # ===================================== Setting the angles and executing each step ===================================== 
            request.name = self.joints_arms
            request.angle = first_arms_help_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Both arms were moved the first time according to the angles ', self.joints_arms, ' with the values: ', first_arms_help_angles,'!', 'OKGREEN'))
            rospy.sleep(3)

            request.angle = second_arms_help_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Both arms were moved a second time according to the angles ', self.joints_arms, ' with the values: ', second_arms_help_angles,'!', 'OKGREEN'))
            rospy.sleep(2)

            return "Result: Pepper requested help with both arms."
        
        # pose 6: Spinning the head
        elif(name == "spin_head"):
            # ===================================== Angles definition for each step ================================================
            # 1. First movement of the head
            first_head_movement_angle = [-0.436332]
            # 2. Second movement of the head
            second_head_movement_angle = [0.436332]
            
            # ===================================== Setting the angles and executing each step ===================================== 
            # 1. First movement of the head
            request.name = ["HeadYaw"]
            request.angle = first_head_movement_angle
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Head was moved the first time according to the angle ', self.joints_head, ' with the values: ', first_head_movement_angle,'!', 'OKGREEN'))
            rospy.sleep(1)

            # 2. Second movement of the head
            request.name = ["HeadYaw"]
            request.angle = second_head_movement_angle
            request.speed = 0.08
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Head was moved a second time according to the angle ', self.joints_head, ' with the values: ', second_head_movement_angle,'!', 'OKGREEN'))
            rospy.sleep(3)

            # 3. Go to state request
            pose_request = go_to_poseRequest()
            pose_request.name = "default_head"
            pose_request.speed = 0.1
            self.motion_set_angle_client.call(request)
            print(consoleFormatter.format('Head was moved to the default pose!', 'OKGREEN'))

            return "Result: Pepper spun her head."

        elif(name=="pick_milk_carton"):
            # ===================================== Angles definition for each step ================================================
            
            # 1. First movement of both arms
            first_arms_help_angles = [0.285927, 0.226893, -0.79688, -0.866391, -0.767945, 0.285973, -0.226893, 0.789178, 0.886188, 0.767945]
            
            hands_parameters = [0.872665, 0.872665]
            
            # 2. Second movement of both arms
            second_arms_help_angles = [0.285927, 0.0174533 , -0.79688, -0.866391, -0.767945, 0.285973, -0.0174533, 0.789178, 0.886188, 0.767945]
            
            # ===================================== Setting the angles and executing each step ===================================== 
            
            request.name = self.joint_hands
            request.angle = hands_parameters
            request.speed = 0.2
            self.motion_set_angle_client.call(request)
            #print(consoleFormatter.format('Hands were opened according to the parameters ', self.joint_hands, ' with the values: ', hands_parameters,'!', 'OKGREEN'))
            rospy.sleep(2)
            
            request.name = self.joints_arms
            request.angle = first_arms_help_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            #print(consoleFormater.format('Both arms were moved the first time according to the angles ', self.joints_arms, ' with the values: ', first_arms_help_angles,'!', 'OKGREEN'))
            rospy.sleep(3)

            request.angle = second_arms_help_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            #print(consoleFormatter.format('Both arms were moved a second time according to the angles ', self.joints_arms, ' with the values: ', second_arms_help_angles,'!', 'OKGREEN'))
            rospy.sleep(2)
            
            return "Result: Pepper placed her both arms in the pose specified."

        elif(name=="pick_cereal_box"):
            # ===================================== Angles definition for each step ================================================
            
            # 1. First movement of both arms
            first_arms_help_angles = [0.285927, 0.279253, -0.79688, -0.866391, -0.767945, 0.285973, -0.279253, 0.789178, 0.886188, 0.767945]
            
            hands_parameters = [0.872665, 0.872665]
            
            # 2. Second movement of both arms
            second_arms_help_angles = [0.285927, 0.122173 , -0.79688, -0.866391, -0.767945, 0.285973, -0.122173, 0.789178, 0.886188, 0.767945]
            
            # ===================================== Setting the angles and executing each step ===================================== 
            
            request.name = self.joint_hands
            request.angle = hands_parameters
            request.speed = 0.2
            self.motion_set_angle_client.call(request)
            #print(consoleFormatter.format('Hands were opened according to the parameters ', self.joint_hands, ' with the values: ', hands_parameters,'!', 'OKGREEN'))
            rospy.sleep(2)
            
            request.name = self.joints_arms
            request.angle = first_arms_help_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            #print(consoleFormater.format('Both arms were moved the first time according to the angles ', self.joints_arms, ' with the values: ', first_arms_help_angles,'!', 'OKGREEN'))
            rospy.sleep(3)

            request.angle = second_arms_help_angles
            request.speed = 0.1
            self.motion_set_angle_client.call(request)
            #print(consoleFormatter.format('Both arms were moved a second time according to the angles ', self.joints_arms, ' with the values: ', second_arms_help_angles,'!', 'OKGREEN'))
            rospy.sleep(2)
            
            return "Result: Pepper placed her both arms in the pose specified."

        # If the action name is not recognized, return an error message
        else: 
            return "Check the input: The action name written was NOT recognized!"

    # ================================== GRASP OBJECT ========================================

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
        request = go_to_poseRequest()

        # Predefined lists categorizing objects based on the appropriate grasping strategy
        list_1 = ["fork", "spoon", "knife", "mug", "bottle", "cereal_box", "milk", "tuna",
                  "tomato_soup", "banana", "strawberry_jello", "canned_meat", "sugar"]
        list_2 = ["bowl", "plate" ]
        list_3 = ["mustard"]
        name_object = req.object 
        
        # Determine the grasping strategy based on the object category
        if(name_object in list_1):
            request.name = "small_object_left_hand"
            request.speed = 0.1
            res = self.set_state.call(request)
            return res.result
        
        elif(name_object in list_2):
            request.name = "bowl"
            request.speed = 0.1
            res = self.set_state.call(request)
            return res.result
        
        elif(name_object in list_3):
            request.name = "master"
            request.speed = 0.1
            res = self.set_state.call(request)
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