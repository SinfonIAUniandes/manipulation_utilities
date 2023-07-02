#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import consoleFormatter

# Manipulation msgs
from manipulation_msgs.srv import GoToState, GoToAction, GraspObject

# Pytoolkit msgs
from manipulation_msgs.srv import set_angle_srv

class ManipulationPytoolkit:
    def __init__(self):
        super(PEPPERMANIPULATION,self).__init__()

        rospy.init_node('ManipulationPytoolkit', anonymous=True)
        
        print(consoleFormatter.format('waiting for goToStatePytoolkit service!', 'WARNING'))  
        self.setAngles = rospy.Service("manipulation_utilities/goToStatePytoolkit", GoToStatePytoolkit, self.callbackGoToStatePytoolkit)
        print(consoleFormatter.format('goToStatePytoolkit on!', 'OKGREEN'))  

        print(consoleFormatter.format('waiting for goToActionPytoolkit service!', 'WARNING'))  
        self.setAngles = rospy.Service("manipulation_utilities/goToActionPytoolkit", GoToActionPytoolkit, self.callbackGoToActionPytoolkit)
        print(consoleFormatter.format('goToActionPytoolkit on!', 'OKGREEN'))  

        print(consoleFormatter.format('waiting for graspObjectPytoolkit service!', 'WARNING'))  
        self.setAngles = rospy.Service("manipulation_utilities/graspObjectPytoolkit", GraspObjectPytoolkit, self.callbackGraspObjectPytoolkit)
        print(consoleFormatter.format('graspObjectPytoolkit on!', 'OKGREEN'))  

        print(consoleFormatter.format('waiting for set_angle_srv from pytoolkit!', 'WARNING'))  
        self.motionSetAngleServer = rospy.ServiceProxy("pytoolkit/ALMotion/set_angle_srv", set_angle_srv)
        print(consoleFormatter.format('motionSetAngleServer connected!', 'OKGREEN'))  
        rospy.spin()

    ###################################################### Go to state ######################################################
    
    def goToStatePytoolkit(self, name):
        angle = []
        joints_arms = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "REbowYaw", "RElbowRoll", "RWristYaw"]
        joints_head = ["HeadPitch", "HeadYaw"]
        joint_left_hand = ["LHand"]
        joint_right_hand = ["RHand"]
        
        # Arms
        if(name=="between_both_arms_1"):
            angle = [-0.00380663, 0.349535, 0.00386407, -0.51711, -1.82379, -0.00378216, -0.352371, 0.00378624, 0.00378624, 1.82381]
        elif(name == "between_both_arms_2"):
            angle = [-0.00375922, 0.176294, 0.00380024, -0.86908, -1.82386, -0.00389758, -0.167802, -0.0038681, 0.87481, 1.82387]
        elif(name == "hold_both_arms_1"):
            angle = [0.00385635, 0.00875869, -0.529993, -0.176235, -1.82385, -0.00372891, -0.00874921, 0.522316, 0.182004, 1.82386]
        elif(name == "hold_both_arms_2"):
            angle = [0.430887, 0.00880139, -0.712923, -0.525612, -1.82379, 0.438531, -0.00874419, 0.697761, 0.531296, 1.82379]
        elif(name == "hold_both_arms_3"):
            angle = [0.334408, 0.0352817, -0.707165, -0.863631, -1.84084, 0.342078, -0.0475533, 0.710233, 0.857495, 1.83769]
        elif(name == "carry_left_arm_1"):
            angle = [0.522307,0.00879306,-1.39166, -0.517085, -1.82386, 1.57468, -0.00880191, 1.56713, 0.00879627, 0.230141]
        elif(name == "carry_right_arm_1"):
            angle = [1.56708, 0.00874494, -1.5748, -0.00874093, -0.223482, 0.529978, -0.00872931, 1.3994, 0.531267, 1.81723]

        setAngles = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)  
        try:
            res = setAngles(joints_arms , angle, 0.1)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        # Head
        if(name == "up_head"):
            angle = [-0.4, 0.0]
        elif(name == "down_head"):
            angle = [0.46, 0.0]
        elif(name == "default_head"):
            angle = [0.0, 0.0]

        setAngles = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)  
        try:
            res = setAngles(joints_hands , angle, 0.1)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        # Open/Close Hand
        if(name == "open_left_hand"):
            angle = [1.0]
        elif(name == "close_left_hand"):
            angle = [0.0]
        elif(name == "open_right_hand"):
            angle = [0.0, 0.0]

        setAngles = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)  
        try:
            res = setAngles(joints_hands , angle, 0.1)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def callbackGoToStatePytoolkit(self,req):
        #Args: name of the position
        res = self.goToStatePytoolkit(req.name)
        return res

    ###################################################### Go to action ######################################################

    def goToActionPytoolkit(self, name):
        angle = []
        joints_arms_hands = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "REbowYaw", "RElbowRoll", "RWristYaw", "HipPitch", "HipPitch"]
        joints_arms_hands = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "REbowYaw", "RElbowRoll", "RWristYaw", "HipPitch", "HipPitch", "LHand", "RHand"]
        joints_hip = ["HipPitch"]
        joint_hands = ["LHand", "RHand"]

        if(name=="place_both_arms"):
            # Se agacha
            angle_1 = [-0.3_5]
            setAction_1 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)  

            # Separa los brazos
            angle_2 = [0.00383162, 1562, -0.00378999, -0.00877543, -4.09046e-05, -9.91789e-05, -1562, -9.95662e-05, 0.00881285, 7.63354e-05, 0-5, 0.5]
            setAction_2 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # giras los brazos
            angle_3 = [1.55179, 1.56201, 0.0039061, -0.00881861, 0.00328999, 1.57469, -1.56204, 1.21692e-05, 0.00882497, -9.82796e-05, 0-5, 0.5]
            setAction_3 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Bajas los brazos
            angle_4 = [1.57471, 0.0940139, 0.00371943, -0.00879696, 0.00338298, 1.57466, -0.0797469, 2.59231e-05, 0.0087464, 3.54903e-05, 0-5, 0.5]
            setAction_4 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            try:
                action_1 = setAction_1(joints_hip , angle_1, 0.2)
                rospy.sleep(2)
                action_2 = setAction_2(joints_arms, angle_2, 0.1)
                rospy.sleep(2)
                action_3 = setAction_3(joints_arms, angles_3, 0.25)
                rospy.sleep(0.5)
                action_4 = setAction_4(joints_arms, angles_3, 0.15)
                rospy.sleep(2)
                return "place both arms executed"

            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        
        if(name=="place_left_arm"):
            # Giras brazo
            angle_1 = [0.00383162, 1562, -0.00378999, -0.00877543, -4.09046e-05, -9.91789e-05, -1562, -9.95662e-05, 0.00881285, 7.63354e-05, 1.0, 0.5]
            setAction_1 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Bajas brazo
            angle_2 = [0.796827, 0.00881551, -1.39942, -0.525606, 1.39701, 1.5746, -0.00885075, 1.56701, 0.00882722, 0.230255, 1, 0.5]
            setAction_2 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)
            
            # Abrir mano
            angle_3 = [0.0, 0.5]
            setAction_3 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Girar brazo
            angle_4 = [0.796941, 1.56198, -1.39166, -0.00880171, 1.3969, 1.57448, -0.0089395, 1.56695, 0.00884955, 0.230282]
            setAction_4 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Girar brazo
            angle_5 = [1.56707, 1.56203, -1.39928, -0.00880251, 1.39715, 1.57455, -0.00897639, 1.56704, 0.00898821, 0.230243]
            setAction_5 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Bajar brazo
            angle_5 = [1.56708, 1.56708, 1.56708, -0.00876288, -0.230052, -0.230052, -0.00896259, 1.56709, 0.00902427, 0.230202]
            setAction_5 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            try:
                action_1 = setAction_1(joints_arms_hands , angle_1, 0.2)
                rospy.sleep(2)
                action_2 = setAction_2(joints_arms_hands , angle_2, 0.2)
                rospy.sleep(2)
                action_3 = setAction_2(joint_hands , angle_3, 0.2)
                rospy.sleep(1)
                action_4 = setAction_2(joints_arms , angle_4, 0.2)
                rospy.sleep(2)
                action_5 = setAction_2(joints_arms , angle_5, 0.2)
                rospy.sleep(1.5)
                action_6 = setAction_6(joints_arms , angle_6, 0.2)
                return "place left arm executed"

        if(name=="place_right_arm"):
            # Giras brazo
            angle_1 = [1.56705, 0.0087755, -1.57479, -0.00876018, -0.00327626, 0.529962, -0.00886489, 1.39943, 0.531231, -1397, 0.5, 1.0]
            setAction_1 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Bajas brazo
            angle_2 = [1.5747, 0.00874223, -1.57477, -0.00876009, -0.00328602, 0.781676, -0.00882641, 1.39931, 0.531273, -1.39709]
            setAction_2 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)
            
            # Abrir mano
            angle_3 = [0.5, 0.0]
            setAction_3 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Extender brazo
            angle_4 = [1.57482, 0.00877891, -1.57479, -0.00882567, -0.00329107, 0.781602, -1.56204, 1.57468, 0.00876204, -1.39709]
            setAction_4 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Girar brazo
            angle_5 = [1.57483, 0.00886465, -1.57476, -0.00883808, -0.00334844, 1.56717, -1.56198, 1.57451, 0.00885734, -1.39706]
            setAction_5 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            # Bajar brazo
            angle_6 = [1.57482, 0.00886047, -1.57472, -0.00891162, -0.00343008, 1.56711, -0.00879193, 2.08561, 0.00880696, -0.523587]
            setAction_6 = rospy.ServiceProxy('pytoolkit/ALMotion/set_angle_srv', set_angle_srv)

            try:
                action_1 = setAction_1(joints_arms_hands , angle_1, 0.2)
                rospy.sleep(2)
                action_2 = setAction_2(joints_arms_hands , angle_2, 0.2)
                rospy.sleep(2)
                action_3 = setAction_3(joint_hands , angle_3, 0.2)
                rospy.sleep(1)
                action_4 = setAction_4(joints_arms , angle_4, 0.1)
                rospy.sleep(2)
                action_5 = setAction_5(joints_arms , angle_5, 0.2)
                rospy.sleep(1.5)
                action_6 = setAction_6(joints_arms , angle_6, 0.2)
                return "place right arm executed"

            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

    def callbackGoToActionPytoolkit(self, req):
        #Args: name of the position
        res = self.goToActionPytoolkit(req.name)
        return res

    ###################################################### Grasp object ######################################################

    def graspObjectPytoolkit(self, object):
        list_carry_right_arm_1 = ["fork", "spoon", "knife", "mug", "bottle", "cereal_box", "milk"]
        list_hold_both_arms_3 = ["bowl", "plate" ]

        if(object in list_carry_right_arm_1):
            carry_right_arm_1 = rospy.ServiceProxy('pytoolkit/ALMotion/goToStatePytoolkit', GoToStatePytoolkit)
            try:
                object = carry_right_arm_1("carry_right_arm_1")
                return "carry right arm executed"

            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
        
        else if(object in list_hold_both_arms_3):
            hold_both_arms_3 = rospy.ServiceProxy('pytoolkit/ALMotion/goToStatePytoolkit', GoToStatePytoolkit)
            try:
                object = hold_both_arms_3("hold_both_arms_3")
                return "hold both arms executed"

            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

    def callbackGraspObjectPytoolkit(self, req):
        #Args: name of the position
        res = self.graspObjectPytoolkit(req.object)
        return res