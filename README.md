# manipulation_utilities
This repository contains the basic implementation of a ROS node designed for the manipulation of a Pepper robot. It leverages PyToolkit services to enhance performance by utilizing the robot's native functions.

**Table of Contents**
- [Branches Description](#branches-description)
- [Create Workspace](#create-workspace)
- [Installation](#installation)
- [Environment Variables](#environment-variables)
- [Instructions](#instructions)
- [Services](#services)

## Branches Description
In the *manipulation_utilities* repository, we maintain several branches to manage the development lifecycle and ensure a stable version of our toolkit is always available. Here's an overview of our main branches:

1. *main:* This is the primary branch that hosts the stable and up-to-date version of the manipulation_utilities_pytoolkit. All changes in this branch are fully documented and have been thoroughly tested to ensure reliability and stability for production use. The main branch is the go-to source for the latest stable release.
2. *dev:* The development branch, serves as the active workspace for our development team. Here, new features, enhancements, and experimental functionalities are developed and integrated. While this branch is regularly updated with the latest developments, it's important to note that it may not always be as stable as the main branch. The dev branch offers a glimpse into the ongoing work and upcoming features but should be used with caution in critical applications.

## Create Workspace
```ROS
mkdir -p manipulation_ws/src
```

## Installation
Clone the following repositories into the manipulation_ws/src directory:
```ROS
git clone https://github.com/SinfonIAUniandes/manipulation_msgs.git	
git clone https://github.com/SinfonIAUniandes/manipulation_utilities.git
```

Then build your project.
```ROS
catkin_make
```

Ensure your ROS environment is properly set up with:
```ROS
source .devel/setup.bash
```

## Environment Variables
Manipulation requires the following environment variables.
 ```bash
nano .bashrc
export ROBOT="PEPPER"
export PEPPER_IP=$(HOSTNAME)
export ROS_MASTER_URI=http://$PEPPER_IP:11311
```

## Instructions:
make sure you have previously launched the toolkit and pytoolkit to have access to their functions.

To run the node in a real robot you should to change your environment variables.
```ROS
rosrun manipulation_utilities manipulation_utilities
```
At this moment you can to call the different services.

## Services:
ManipulationUtilities offers the following services:

### 1. go_to_state:

+ ***Description:*** Executes a specific pose for the Pepper robot based on the provided request, which includes the name of the desired pose and the speed at which the movement should be performed.

+ ***Service file:*** *go_to_state.srv*
    + ***Request***: 
		+ name (std_msgs/String): The name of the pose or movement to execute, corresponding to an entry in the 'objects_poses.csv' file.
    + speed (std_msgs/Float32): The speed at which the joints should move to reach the specified angles.
	+ ***Response***:
		+ result (std_msgs/Bool): Indicates whether the pose was successfully executed.
		
+ ***Call service example:***
 ```bash
rosservice call /manipulation_utilities/go_to_state "name: 'pose_name' speed: 0.5"
```

### 2. go_to_action:

+ ***Description:*** Performs a predefined action based on the requested action name, such as moving arms or grasping objects, with predetermined joint angles and speeds.

+ ***Service file:*** *go_to_action.srv*
    + ***Request***: 
		+ name (std_msgs/String): The name of the action to be performed, like 'place_both_arms', 'place_left_arm', etc.
	+ ***Response***:
		+ feedback (std_msgs/String): A message providing feedback about the executed action or indicating if the action name was not recognized.
		
+ ***Call service example:***
 ```bash
rosservice call /manipulation_utilities/go_to_action "name: 'place_both_arms'"
```

### 3. grasp_object::

+ ***Description:*** Handles a grasping request for various objects by categorizing them and assigning a specific pre-defined state for the Pepper to execute, based on the object type.

+ ***Service file:*** *grasp_object.srv*
    + ***Request***: 
		+ object (std_msgs/String): The name of the object to be grasped.
	+ ***Response***:
		+ result (std_msgs/String): The result of the service call, providing feedback on the grasping action.
		
+ ***Call service example:***
 ```bash
rosservice call /manipulation_utilities/grasp_object "object: 'bottle'"
```

### 4. move_head::

+ ***Description:*** Controls the movement of the Pepper's head to the specified angles, ensuring the angles are within safe limits for movement.
  
+ ***Service file:*** *move_head.srv*
    + ***Request***: 
		+ angle1 (std_msgs/Float32): Desired angle for 'HeadPitch', specified in degrees.
    + angle2 (std_msgs/Float32): Desired angle for 'HeadYaw', also in degrees.
	+ ***Response***:
		+ result (std_msgs/String): Provides feedback on the head movement or an error message if the desired angles are not achievable.
		
+ ***Call service example:***
 ```bash
rosservice call /manipulation_utilities/move_head "angle1: 10 angle2: 15"
```
