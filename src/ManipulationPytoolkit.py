#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import ConsoleFormatter

# Manipulation msgs
from manipulation_msgs_pytoolkit.srv import GoToState, GoToAction, GoToActionRequest, GoToStateRequest, GraspObject

# Pytoolkit msgs
from manipulation_msgs_pytoolkit.srv import set_angle_srv, set_angle_srvRequest

class ManipulationPytoolkit:
    def __init__(self):

        rospy.init_node('ManipulationPytoolkit', anonymous=True)
        
        print(consoleFormatter.format('waiting for goToStatePytoolkit service!', 'WARNING'))  
        self.goToState= rospy.Service("manipulation_utilities/goToState", GoToState, self.callbackGoToStatePytoolkit)
        print(consoleFormatter.format('goToStatePytoolkit on!', 'OKGREEN'))  

        print(consoleFormatter.format('waiting for goToActionPytoolkit service!', 'WARNING'))  
        self.goToAction = rospy.Service("manipulation_utilities/goToAction", GoToAction, self.callbackGoToActionPytoolkit)
        print(consoleFormatter.format('goToActionPytoolkit on!', 'OKGREEN'))  

        print(consoleFormatter.format('waiting for graspObjectPytoolkit service!', 'WARNING'))  
        self.graspObject = rospy.Service("manipulation_utilities/graspObject", GraspObject, self.callbackGraspObjectPytoolkit)
        print(consoleFormatter.format('graspObjectPytoolkit on!', 'OKGREEN'))  

        print(consoleFormatter.format('waiting for goToStatePytoolkit service!', 'WARNING'))  
        self.setState = rospy.ServiceProxy("manipulation_utilities/goToState", GoToState)
        print(consoleFormatter.format('goToStatePytoolkit connected!', 'OKGREEN'))  

        print(consoleFormatter.format('waiting for set_angle_srv from pytoolkit!', 'WARNING'))  
        self.motionSetAngleClient = rospy.ServiceProxy("pytoolkit/ALMotion/set_angle_srv", set_angle_srv)
        print(consoleFormatter.format('motionSetAngleServer connected!', 'OKGREEN'))  

    ###################################################### Go to state ######################################################

    def callbackGoToStatePytoolkit(self,req):
        request = set_angle_srvRequest()
        name = req.name
        velocity = req.velocity
        angle = []
        joints_arms = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        joints_left_arm = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        joints_right_arm = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        joints_head = ["HeadPitch", "HeadYaw"]
        joint_left_hand = ["LHand"]
        joint_right_hand = ["RHand"]
        joint_hands = ["LHand", "RHand"]

        # Name joints
        if name == "box" or name == "cylinder" or  name== "tray" or name == "medium_object" or name == "bowl" or name == "bottle":
            request.name = joints_arms
        elif name == "small_object_left_hand":
            request.name = joints_left_arm
        elif  name == "small_object_right_hand":
            request.name = joints_right_arm
        elif name == "up_head" or name == "up_down" or name == "default_head":
            request.name =  joints_head
        elif name == "open_left_hand" or name == "open_left_hand":
            request.name = joint_left_hand
        elif name == "open_right_hand" or name == "open_right_hand":   
            request.name = joint_right_hand 
        elif name == "open_both_hands" or name == "close_both_hands":  
            request.name = joint_hands
        else: 
            return "State not found"

        # Arms
        if(name=="box"):
            angle = [-0.00380663, 0.349535, 0.00386407, -0.51711, -1.82379, -0.00378216, -0.352371, 0.00378624, 0.00378624, 1.82381]
        elif(name == "cylinder"):
            angle = [-0.00375922, 0.176294, 0.00380024, -0.86908, -1.82386, -0.00389758, -0.167802, -0.0038681, 0.87481, 1.82387]
        elif(name == "tray"):
            angle = [0.00385635, 0.00875869, -0.529993, -0.176235, -1.82385, -0.00372891, -0.00874921, 0.522316, 0.182004, 1.82386]
        elif(name == "medium_object"):
            angle = [0.430887, 0.00880139, -0.712923, -0.525612, -1.82379, 0.438531, -0.00874419, 0.697761, 0.531296, 1.82379]
        elif(name == "bowl"):
            angle = [0.334408, 0.0352817, -0.707165, -0.863631, -1.84084, 0.342078, -0.0475533, 0.710233, 0.857495, 1.83769]
        elif(name == "small_object_left_hand"):
            angle = [0.522307,0.00879306,-1.39166, -0.517085, -1.82386]
        elif(name == "small_object_right_hand"):
            angle = [1.56708, 0.00874494, -1.5748, -0.00874093, -0.223482]
        elif(name == "bottle"):
            angle = [0.781723, 0.00873155, -0.690057, -0.871938, -1.82384, 0.324122, -0.00874611, 0.16388, 0.579455, 1.82377]

        # Head
        elif(name == "up_head"):
            angle = [-0.4, 0.0]
        elif(name == "down_head"):
            angle = [0.46, 0.0]
        elif(name == "default_head"):
            angle = [0.0, 0.0]

        # Left Hand
        elif(name == "open_left_hand"):
            angle = [1.0]
        elif(name == "close_left_hand"):
            angle = [0.0]
        
        # Right Hand
        elif(name == "open_right_hand"):
            angle = [1.0]
        elif(name == "close_right_hand"):
            angle = [0.0]

        # Open/Close hand 
        if(name == "open_both_hands"):
            angle = [1.0, 1.0]
        elif(name == "close_both_hands"):
            angle = [0.0, 0.0]

        request.angle = angle
        request.speed = velocity
        res = self.motionSetAngleClient.call(request)
        return res.result

    ###################################################### Go to action ######################################################

    def callbackGoToActionPytoolkit(self, req):
        request = set_angle_srvRequest()
        name = req.name
        angle = []

        joints_arms_hands = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LHand", "RHand"]
        joints_left_arm = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        joints_right_arm = ["RShoulderPitch", "R1ShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]    
        joints_arms = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        joints_hip = ["HipPitch"]
        joint_hands = ["LHand", "RHand"]
        joint_left_hand = ["LHand"]
        joint_right_hand = ["RHand"]

        if(name=="place_both_arms"):
            # Se agacha
            angle_1 = [-0.35]

            # Abre las manos
            angle_2 = [1.0, 0.5]

            # Separa los brazos
            angle_3 = [0.00383162, 3.8160, -0.00378999, -0.00877543, -4.09046e-05, -9.91789e-05, -3.8160, -9.95662e-05, 0.00881285, 7.63354e-05, 0.5, 0.5]

            # giras los brazos
            angle_4 = [1.55179, 1.56201, 0.0039061, -0.00881861, 0.00328999, 1.57469, -1.56204, 1.21692e-05, 0.00882497, -9.82796e-05, 0-5, 0.5]

            # Bajas los brazos
            angle_5 = [1.57471, 0.0940139, 0.00371943, -0.00879696, 0.00338298, 1.57466, -0.0797469, 2.59231e-05, 0.0087464, 3.54903e-05, 0-5, 0.5]

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
            return "place both arms executed"
        
        elif(name=="place_left_arm"):
            # Giras brazo
            angle_1 = [0.522444, 0.00885305, -1.39163, -0.517197, 1.57039]

            # Bajas brazo
            angle_2 = [0.796827, 0.00881551, -1.39942, -0.525606, 1.39701]
            
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
            res = self.motionSetAngleClient.call(request)

            request.angle = angle_6
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(1.5)

            return "place left arm executed"

        elif(name=="place_right_arm"):
            # Giras brazo
            angle_1 = [0.529962, -0.00886489, 1.39943, 0.531231, -2.174]

            # Bajas brazo
            angle_2 = [0.781676, -0.00882641, 1.39931, 0.531273, -1.39709]
            
            # Abrir mano
            angle_3 = [1.0]

            # Extender brazo
            angle_4 = [0.781602, -1.56204, 1.57468, 0.00876204, -1.39709]

            # Girar brazo
            angle_5 = [-0.00334844, 1.56717, -1.56198, 1.57451, 0.00885734, -1.39706]

            # Bajar brazo
            angle_6 = [1.56711, -0.00879193, 2.08561, 0.00880696, -0.523587]

            request.name = joints_right_arm
            request.angle = angle_1
            request.speed = 0.2
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.angle = angle_2
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.name = joint_right_hand
            request.angle = angle_3
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(1)

            request.name = joints_right_arm
            request.angle = angle_4
            request.speed = 0.1
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(2)

            request.angle = angle_5
            request.speed = 0.2
            res = self.motionSetAngleClient.call(request)

            request.angle = angle_6
            request.speed = 0.2
            res = self.motionSetAngleClient.call(request)
            rospy.sleep(1.5)
            return "place right arm executed"
        else: 
            return "No action name recognized"


    ###################################################### Grasp object ######################################################

    def callbackGraspObjectPytoolkit(self, req):
        request = GoToStateRequest()

        list_1 = ["fork", "spoon", "knife", "mug", "bottle", "cereal_box", "milk"]
        list_2 = ["bowl", "plate" ]
        name_object = req.object 
        
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
        
        else:
            return "Error"

if __name__ == '__main__':
    consoleFormatter=ConsoleFormatter.ConsoleFormatter()
    manipulationPytoolkit = ManipulationPytoolkit()
    try:
        print(consoleFormatter.format(" --- manipulation utilities node successfully initialized ---","OKGREEN"))
        rospy.spin()

    except rospy.ROSInterruptException:
        pass