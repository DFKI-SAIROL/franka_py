- [1. frankapy](#1-frankapy)
  - [1.1. Info - Main modules](#11-info---main-modules)
    - [1.1.1. franka\_launch](#111-franka_launch)
    - [1.1.2. franka\_meta\_quest](#112-franka_meta_quest)
    - [1.1.4. franka\_safety\_layer](#114-franka_safety_layer)
  - [1.2. Info - Helper and old modules](#12-info---helper-and-old-modules)
    - [1.2.1. franka\_adaptable\_controllers](#121-franka_adaptable_controllers)
    - [1.2.2. franka\_control\_module](#122-franka_control_module)
    - [1.2.3. franka\_custom\_msgs](#123-franka_custom_msgs)
    - [1.2.4. franka\_visulazation](#124-franka_visulazation)
  - [1.3. Starting the nodes](#13-starting-the-nodes)
  - [1.4. Installation](#14-installation)


# 1. frankapy

This package aims to simplify the use of the franka research 3 robots by providing controllers, launch and configuration files for single and bimanual setup

In the whole repository three namespaces are used. For the single setup NS='franka_main' an for the bimanual setup its 'franka_left' and 'franka_right'.

The current development is still not finished and only partly tested for the bimanual setup, there may be some small changes required to make the single setup work.

## 1.1. Info - Main modules

### 1.1.1. franka_launch

franka_launch is our customized version of franka_bringup and contains the launch and configuration files for starting the robot.

To start the robot in simulation, the parameter ```use_fake_hardware:=true``` can be used.
However it is worth noting that the robot behaves fundamentally differently in simulation. Most of the commands check are not evaluated in this mode and therefore developing the robot's control in simulation does not work very well. Higher-level software can be tested in simulation once the lower-level control is working on the robot.

The robot simulation provided by frankarobotics only supports position and velocity control. Therefore, when using the simulation, in the controllers.yaml the command_interfaces of the desired controller needs to be changed from "effort" to "position". Else the robot wont move. 

### 1.1.2. franka_meta_quest

franka_meta_quest contains the python scripts for teleoperation using the meta quest.
This is adapted from the https://github.com/droid-dataset/droid repository.
Currently it publishes the teleoperated target pose using a geometry_msgs.msg.PoseStamped on the topic 'NS/target_cartesian_pose'.

For the tracking of the joysticks to work well, it is important that the meta quest is placed stationary such that the joysticks are visible by its camera system.


### 1.1.4. franka_safety_layer

This package contains the safety layer that enables the safe execution of cartesian poses (and twists). This can be used to allow data collection / teleoperation, model training and inference on the robot. 
For this, the cartesian target is checked for safety violations and adjusted and then joint commands are computed with an Inverse Jacobian with Nullspace-Control approach.
The joint commands are interpolated and executed by the franka_joint_trajectory_controller.

## 1.2. Info - Helper and old modules

### 1.2.1. franka_adaptable_controllers

This package was used to test the robot's position control interface by adapting one of the provided franka_example_controllers to accept target positions with a ros2 subscriber and manually interpolating between the last and new command. We found that using the franka_joint_trajectory_controller is the better way to do this and do not use this package anymore.

### 1.2.2. franka_control_module 

This package contains two nodes which can publish simple motions in the cartesian or joint space for testing and debugging.

franka_control_module started by start.launch.py is used to publish joint commands for checking basic movements of the franka_joint_trajectory_controller and the robot.

franka_cartesian_module started by cartesian.launch.py publishes figure 8 movements in cartesian space and is used during the development of the safety layer for testing the inverse jacobian approach and the safety bounds.

### 1.2.3. franka_custom_msgs

This package defines custom ros2 messages used for debugging the nodes, for example using plotjuggler.

### 1.2.4. franka_visulazation

The visualization node draws a visualization_msgs::Marker line and an arrow to visualize the endeffectors pose and movement.
For this is subscibes to the cartesian_target_pose of the teleoperation and the "safe cartesian target pose" adjusted by the safety layer and the actual tf pose of the endeffector and visualizes all three using an arrow to show the current pose and a line to show the recent positions. 


## 1.3. Starting the nodes

The robot can be started using ``` ros2 launch franka_launch example.launch.py spawn_franka_left:=<false|true>     spawn_franka_right:=<true|true>     use_fake_hardware:=<false|true> ```.

The safety-layer-node can be started using ``` ros2 launch franka_safety_layer start_ijk.launch.py spawn_franka_left:=<false|true> spawn_franka_right:=<false|true> bypass_safety:=<false|true>```. 

The teleoperation-node can be started using ``` ros2 launch franka_meta_quest start.launch.py ``` when the meta quest is connected via usb and usb-debugging is enabled on the meta quest. 
For testing, the cartesian-control-module-node can be started using ``` ros2 launch franka_control_module cartesian.launch.py ```. 

The visualization can be started using ``` ros2 launch franka_visualization pose_lines.launch.py ```.

In the current development stage it is important to be able to start and restart different nodes in seperate terminals individually, in a later stage the launch files can be merged to simplify starting the system.

## 1.4. Installation

For the installation, please follow the steps in the [officical franka ros2 repo](https://github.com/frankarobotics/franka_ros2/blob/jazzy/README.md).

Furthermore, for the meta quest communication the "ABD-Client" needs to be installed using ```pip install pure-python-adb```, ideally in a conda environment.

TODO extended installation documentation for the conda env conaining everything from ros2 to pinocchio, pure-python-adb. 
