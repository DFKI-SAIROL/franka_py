# Launch files for Franka Fr3s

## Launching robots

We usually launch the robots from `dynamic_example.launch.py`.

```
ros2 launch franka_launch example.launch.py \
    robot_config_file:=dfki_bimanual \
    spawn_franka_left:=false \
    spawn_franka_right:=true \
    use_fake_hardware:=true
```

## Launching grippers 

Grippers are launched independently from the arms. We currently support the usual Franka parallel grippers and the Robotis RH-P12-RN-A grippers

### Launching Robotis (RH-P12-RN-A) gripper

Before launching, make sure that the gripper is connected to a power source and that the switch on the controller board is switched on.

Afterwards, you can launch the Robotis gripper with:

```
ros2 launch franka_launch gripper.launch.py arm_prefix:=franka_right
```

> TODO: The arm_prefix is important as it is currently hardcoded as the prefix of the joint states in the `dynamixel_controllers.yaml`


You can check if the controller is working by publishing joint position commands to the 

```
ros2 topic pub --once /franka_right/gripper/gripper_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    "{joint_names: ['franka_right_rh_r1'], points: [{positions: [0.5], time_from_start: {sec: 1, nanosec: 0}}]}"
```

The position ranges ranges are [0.0, 1.1] (open, closed).

### Launching Franka gripper

> TODO: Not implemented yet!