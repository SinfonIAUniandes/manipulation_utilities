#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import ConsoleFormatter

# Manipulation msgs
from manipulation_msgs_pytoolkit.srv import GoToState, GoToAction, GoToActionRequest, GoToStateRequest, GraspObject
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

# Pytoolkit msgs
from manipulation_msgs_pytoolkit.srv import set_angle_srv, set_angle_srvRequest, set_stiffnesses_srv, set_stiffnesses_srvRequest

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

        # Off autonomous life 
        print(consoleFormatter.format('waiting for set_state_srv from pytoolkit!', 'WARNING'))  
        self.motionSetStatesClient = rospy.ServiceProxy("pytoolkit/ALAutonomousLife/set_state_srv", SetBool)
        print(consoleFormatter.format('set_state_srv connected!', 'OKGREEN')) 

        # On Stiffness in robot
        print(consoleFormatter.format('waiting for set_stiffness_srv from pytoolkit!', 'WARNING'))  
        self.motionSetStiffnessesClient = rospy.ServiceProxy("pytoolkit/ALMotion/set_stiffnesses_srv", set_stiffnesses_srv)
        print(consoleFormatter.format('set_stiffness_srv connected!', 'OKGREEN')) 
        
        self.initialize()
    
    def initialize(self):

        # Off autonomous life 
        req_states = SetBoolRequest()
        req_states.data = False
        #res = self.motionSetStatesClient.call(req_states)

        # On Stiffness in robot
        req_stiffnesses = set_stiffnesses_srvRequest()

        req_stiffnesses.names = "RHand"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "LHand"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "LShoulderPitch"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "LShoulderRoll"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "LElbowYaw"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "LElbowRoll"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "LWristYaw"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "RShoulderPitch"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "RShoulderRoll"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "RElbowYaw"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "RElbowRoll"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)

        req_stiffnesses.names = "RWristYaw"
        req_stiffnesses.stiffnesses = 1
        res = self.motionSetStiffnessesClient.call(req_stiffnesses)
        
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
        if name == "box" or name == "bowl_pequeño2" or name == "bowl_pequeño" or name == "master" or name == "pringles" or name == "cylinder" or  name== "tray" or name == "medium_object" or name == "bowl" or name == "bottle" or name == "standard" or name == "tray_full":
            request.name = joints_arms
        elif name == "small_object_left_hand":
            request.name = joints_left_arm
        elif  name == "small_object_right_hand":
            request.name = joints_right_arm
        elif name == "up_head" or name == "down_head" or name == "default_head":
            request.name =  joints_head
        elif name == "open_left_hand" or name == "close_left_hand" or name == "almost_open_left_hand" or name == "almost_close_left_hand":
            request.name = joint_left_hand
        elif name == "open_right_hand" or name == "close_right_hand" or name == "almost_open_right_hand" or name == "almost_close_right_hand":
            request.name = joint_right_hand 
        elif name == "open_both_hands" or name == "close_both_hands" or name == "almost_open_both_hands" or name == "almost_close_both_hands":  
            request.name = joint_hands
        else: 
            return "State not found"

        # Arms
        if(name=="box"):
            angle = [-0.00380663, 0.349535, 0.00386407, -0.51711, -1.82379, -0.00378216, -0.352371, 0.00378624, 0.528438, 1.82381]
        elif(name == "cylinder"):
            angle = [-0.00375922, 0.176294, 0.00380024, -0.86908, -1.82386, -0.00389758, -0.167802, -0.0038681, 0.87481, 1.82387]
        elif(name == "tray"):
            angle = [0.00385635, 0.00875869, -0.529993, -0.176235, -1.82385, -0.00372891, -0.00874921, 0.522316, 0.182004, 1.82386]
        elif(name == "medium_object"):
            angle = [0.430887, 0.00880139, -0.712923, -0.525612, -1.82379, 0.438531, -0.00874419, 0.697761, 0.531296, 1.82379]
        elif(name == "bowl"):
            angle = [0.796985, 0.00881456, -0.707278, -0.863581, -1.82383, 0.789256, -0.00880797, 0.71025, 0.857399, 1.82379]
        elif(name == "small_object_left_hand"):
            angle = [0.10472, -0.0523599, -1.39626, -0.191986, -1.81514]
        elif(name == "small_object_right_hand"):
            angle = [0.10472, -0.0523599, 1.39626, 0.191986, 1.81514]
        elif(name == "bottle"):
            angle = [0.781723, 0.00873155, -0.690057, -0.871938, -1.82384, 0.324122, -0.00874611, 0.16388, 0.579455, 1.82377]
        elif(name == "pringles"):
            angle = [0.514665, 0.0484047, -0.781569, -0.721599, -1.82386, 0.507025, -0.0570473, 0.796853, 0.710169, 1.82379]
        elif(name == "standard"):
            angle = [1.56717, 0.00878502, -1.56704, -0.00877923, -9.88085e-06, 1.55183, -0.00874544, -0.00385945, 0.00877765, 1.82379]
        elif(name == "tray_full"):
            angle = [1.04842, 0.0087904, -1.57465, -1.00254, -1.56368, 1.04853, -0.00881849, 1.57481, 0.999722, 1.56373]
        elif(name == "master"):
            angle = [0.865441, 0.00884603, -0.804491, -0.997038, -0.623608, 0.873268, -0.0088472, 0.796842, 0.994103, 0.616864]
        elif(name == "bowl_pequeño"):
            angle = [0.316383, 0.00873112, -0.690055, -0.786829, -1.82382, 0.308927, -0.00874898, 0.690114, 0.792578, 1.82378]
        elif(name == "bowl_pequeño2"):
            angle = [0.285927, 0.088337, -0.79688, -0.866391, -1.82384, 0.285973, -0.0939549, 0.789178, 0.886188, 1.82378]

        # Head
        elif(name == "up_head"):
            angle = [-0.45, 0.0]
        elif(name == "down_head"):
            angle = [0.46, 0.0]
        elif(name == "default_head"):
            angle = [0.0, 0.0]

        # Left Hand
        elif(name == "open_left_hand"):
            angle = [1.0]
        elif(name == "almost_open_left_hand"):
            angle = [0.75]
        elif(name == "close_left_hand"):
            angle = [0.0]
        elif(name == "almost_close_left_hand"):
            angle = [0.25]

        # Right Hand
        elif(name == "open_right_hand"):
            angle = [1.0]
        elif(name == "almost_open_right_hand"):
            angle = [0.75]
        elif(name == "close_right_hand"):
            angle = [0.0]
        elif(name == "almost_close_right_hand"):
            angle = [0.25]

        # Open/Close hand 
        if(name == "open_both_hands"):
            angle = [1.0, 1.0]
        elif(name == "almost_open_both_hands"):
            angle = [0.75, 0.75]
        elif(name == "close_both_hands"):
            angle = [0.0, 0.0]
        elif(name == "almost_close_both_hands"):
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

    ############################v########################## Grasp object ######################################################

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