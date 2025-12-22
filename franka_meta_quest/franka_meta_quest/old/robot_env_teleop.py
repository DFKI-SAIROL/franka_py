import os
from copy import deepcopy

import gym
import numpy as np
import logging

#from droid.calibration.calibration_utils import load_calibration_info
# from droid.camera_utils.info import camera_type_dict
#from droid.camera_utils.wrappers.multi_camera_wrapper import MultiCameraWrapper
from droid.misc.parameters import gripper_cam_id, nuc_ip
from droid.misc.server_interface import ServerInterface
from droid.misc.time import time_ms
#from droid.misc.transformations import change_pose_frame
from droid.misc.transformations import add_angles, euler_to_quat, quat_diff, quat_to_euler, rmat_to_quat

from franky import Gripper, Robot, Affine, CartesianMotion, ReferenceType, JointVelocityMotion, JointMotion, JointWaypointMotion, JointWaypoint, CartesianVelocityMotion, Twist, RelativeDynamicsFactor, Duration, Reaction, Measure, ControlException
# import matplotlib.pyplot as plt
from collections import deque
import time
from scipy.spatial.transform import Rotation

#from .gripper import Gripper
#import traceback


class RobotEnv(gym.Env):
    def __init__(self, action_space="cartesian_velocity", gripper_action_space=None, camera_kwargs={}, do_reset=True):
        # Initialize Gym Environment
        super().__init__()

        # Define Action Space #
        assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
        self.action_space = action_space
        #self.gripper_action_space = gripper_action_space
        self.check_action_range = "velocity" in action_space

        # Robot Configuration
        self.reset_joints = np.array([0.8, 0.0, -0.2, -1.3, 0.0, 1.8, 0.8]) # np.array([0, -1 / 5 * np.pi, 0, -4 / 5 * np.pi, 0, 3 / 5 * np.pi, 0.0])
        self.randomize_low = np.array([-0.1, -0.2, -0.1, -0.3, -0.3, -0.3])
        self.randomize_high = np.array([0.1, 0.2, 0.1, 0.3, 0.3, 0.3])
        self.DoF = 7 # 7 if ("cartesian" in action_space) else 8
        self.control_hz = 15

        self.prev_action_lin = np.zeros(3)

        # Create Cameras
        #self.camera_reader = MultiCameraWrapper(camera_kwargs)
        #self.calibration_dict = load_calibration_info()

        time.sleep(1)

        if True: #nuc_ip is None:
            # from droid.franka.robot import FrankaRobot

            # try:
            #     self.gripper = Gripper()
                
            # except Exception as e:
            #     logging.warning("WARNING: Gripper NOT INITIALIZED")
            #     print(e)
            #     self.gripper = None


            # self._robot = FrankaRobot()
            self.robot = Robot("192.168.0.2", default_force_threshold=100, default_torque_threshold=100)
            self.robot.recover_from_errors()
            self.robot.set_joint_impedance([700] * 7)
            # self.gripper.open()

            # motion = CartesianMotion(Affine([0.0, 0.0, -0.01]), ReferenceType.Relative)
            # self.robot.move(motion)
            # motion = CartesianMotion(Affine([0.0, 0.0, 0.01]), ReferenceType.Relative)
            # self.robot.move(motion)

            # self.gripper = Gripper("192.168.0.4")
            # self.gripper_speed = 0.02  # [m/s]
            # self.gripper_force = 20.0  # [N]
            # self.last_action = None

            # gripper.open(speed)
            # # Move the fingers to a specific width (5cm)
            # success = gripper.move(0.05, speed)


        else:
            raise NotImplementedError
            self._robot = ServerInterface(ip_address=nuc_ip)

        # Reset Robot
        if do_reset:
            self.reset()
        
        # self.camera_type_dict = camera_type_dict
        
        # _, self.ax = plt.subplots(3)
        # self.plt = [(ax.plot([], [], label="orig")[0], ax.plot([], [], label="filtered")[0]) for ax in self.ax]
        # plt.legend()
        # self.futures = deque()
        # plt.show(block=False)

    def step(self, action):
        # Check Action
        assert len(action) == self.DoF
        # if self.check_action_range:
        #     assert (action.max() <= 1) and (action.min() >= -1)

        # Update Robot
        action_info = self.update_robot(
            action,
            action_space=self.action_space,
            gripper_action_space=None,
        )

        # Return Action Info
        return action_info

    def reset(self, randomize=False):
        # self.gripper.open()

        self.prev_action_lin = np.zeros(3)

        if randomize:
            noise = np.random.uniform(low=self.randomize_low, high=self.randomize_high)
        else:
            noise = None

        #self._robot.update_joints(self.reset_joints, velocity=False, blocking=True, cartesian_noise=noise)
        from franky import JointWaypointMotion, JointWaypoint

        try:
            self.robot.join_motion()
        except Exception as e:
            print(f"WARNING: {e}")

        reset_move = JointWaypointMotion([JointWaypoint([0.8, 0.0, -0.2, -1.6, 0.0, 1.8, 0.8])], relative_dynamics_factor=0.1)
        self.robot.recover_from_errors()
        try:
            self.robot.move(reset_move)
        except ControlException as ex:
            print(ex)
            self.robot.recover_from_errors()

        try:
            self.robot.join_motion()
        except ControlException as ex:
            print(ex)
            self.robot.recover_from_errors()


    def update_robot(self, action, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        if action_space == "joint_position":

            joint_action, gripper_action = action[:7], action[-1]
            #self.gripper.move_to(gripper_action)
            self.robot.move(JointMotion(joint_action, relative_dynamics_factor=0.1), asynchronous=not blocking)

        elif action_space == "joint_velocity":

            joint_action, gripper_action = action[:7], action[-1]
            #self.gripper.velocity_control(gripper_action)
            try:
                self.robot.move(JointVelocityMotion(
                    joint_action,
                    duration=Duration(int(4 / self.control_hz * 1000)), 
                    relative_dynamics_factor=RelativeDynamicsFactor(velocity=0.1, acceleration=0.1, jerk=0.1),
                    # state_estimate_weight=0.0
                ), asynchronous=not blocking)
            except ControlException as ex:
                print(ex)
                self.robot.recover_from_errors()

        elif action_space == "cartesian_velocity":

            # action_info = self._robot.update_command(
            #     action,
            #     action_space=action_space,
            #     gripper_action_space=gripper_action_space,
            #     blocking=blocking
            # )

            # if action_space == "cartesian_velocity":
            #     motion = CartesianMotion(Affine(action), ReferenceType.Relative)

            # print(action)

            # current_joints = self.robot.state.q

            # current_action = action.copy()



            # if self.last_action is None:
            #     self.last_action = np.zeros_like(action)
            # else:
            #     action = action - self.last_action
            #     self.last_action = current_action 

            # self.last_action = action.copy()

            lin_vel, rotation, gripper_velocity =  action[:3], action[3:-1], action[-1]

            alpha = 0.9
            self.prev_action_lin = self.prev_action_lin * alpha + (1 - alpha) * lin_vel

            # print(gripper_pos)

            # os.system('clear')
            # print("present_pwm", self.gripper.connector.read_field("present_pwm"))
            # print("present_current", self.gripper.connector.read_field("present_current"))
            # print("present_velocity", self.gripper.connector.read_field("present_velocity"))
            # print("present_position", self.gripper.connector.read_field("present_position"))
            # print("profile_velocity", self.gripper.connector.read_field("profile_velocity"))
            # print()

            # TODO: add again
            #self.gripper.velocity_control(gripper_velocity)

            # self.gripper.move_to(gripper_pos)

            # self.futures.append(self.gripper.grasp_async(self.gripper.width + gripper, self.gripper_speed, self.gripper_force, epsilon_outer=1.0))
            # start = time.time()
            # # thread = Thread(target=self.gripper.move, args=(self.gripper.width + gripper, self.gripper_speed))
            # # thread.start()_update_internal_state
    
            #print(translation)

            #translation = 0.2 * translation

            # translation[0] = 0.0
            # translation[1] = 0.0
            # translation[2] = 0.0

            #x = Affine(translation)
            #x = self.robot.current_pose * Affine(translation) #* Affine([0.0, 0.0, 0.0], euler_to_quat(rotation))

            #print(x)

            # action[-2:] + 0.0
            # target_joints = action 
            # target_joints = current_joints + 0.1 * action 
            # dt = 1 / self.control_hz
            # rotation_delta = Rotation.from_rotvec(rotation * dt)
            # rotation_delta_affine = Affine(quaternion=rotation_delta.as_quat())
            # target_rotation_affine = self.robot.current_pose.end_effector_pose * rotation_delta_affine
            
            # motion = CartesianMotion(Affine(np.array(lin_vel) * dt + self.robot.current_pose.end_effector_pose.translation, target_rotation_affine.quaternion))
            # motion = JointWaypointMotion([JointWaypoint(target_joints)])
            # motion = CartesianMotion(Affine(np.array(lin_vel) * dt + self.robot.current_pose.end_effector_pose.translation, euler_to_quat([np.pi, 0, 0])))
            robot_rot = Rotation.from_quat(self.robot.current_pose.end_effector_pose.quaternion)
            # from franky import RobotVelocity
            # motion = CartesianVelocityMotion(
            #     RobotVelocity(Twist(np.array(lin_vel), robot_rot.apply(rotation))), 
            #     duration=Duration(int(4 / self.control_hz * 1000)), 
            #     relative_dynamics_factor=RelativeDynamicsFactor(velocity=0.1, acceleration=0.1, jerk=0.1),
            #     state_estimate_weight=np.zeros((3,1)))
            
            # NOTE: Committed command, getting errors with it!
            motion = CartesianVelocityMotion(
               Twist(np.array(lin_vel), robot_rot.apply(rotation)), 
               duration=Duration(int(4 / self.control_hz * 1000)), 
               relative_dynamics_factor=RelativeDynamicsFactor(velocity=0.1, acceleration=0.1, jerk=0.1),
            #    state_estimate_weight=0.0
               )

            # reaction = Reaction((Measure.FORCE_X ** 2 + Measure.FORCE_Y ** 2 + Measure.FORCE_Z ** 2) ** 0.5 > 20, CartesianVelocityMotion(Twist()))
            # reaction.register_callback(lambda *args, **kwargs: print("Reaction"))
            # motion.add_reaction(reaction)
            # motion.register_callback(lambda rs, t, b, c, d: None)
            # motion = JointWaypointMotion([JointWaypoint(target_joints)])
                
            try:
                self.robot.move(motion, asynchronous=not blocking)
            except ControlException as ex:
                print(ex)
                self.robot.recover_from_errors()

            # for i, (p, ax) in enumerate(zip(self.plt, self.ax)):
            #     new_data = np.concatenate([p[0].get_ydata(), [lin_vel[i]]])[-100:]
            #     p[0].set_data(
            #         np.arange(len(new_data)),
            #         new_data
            #         )
            #     # new_data = np.concatenate([p[1].get_ydata(), [self.prev_action_lin[i]]])[-100:]
            #     # p[1].set_data(
            #     #     np.arange(len(new_data)),
            #     #     new_data
            #     #     )
            #     ax.relim()
            #     ax.set_ylim(-0.5, 0.5)
            #     ax.autoscale_view()
            # plt.pause(0.001)

                # self.robot.move(motion, asynchronous=not blocking)
        elif action_space == "cartesian_position":
            translation = action[:3]
            rotation = action[3:-1]
            quat = Rotation.from_euler("xyz", rotation).as_quat()
            motion = CartesianMotion(
                Affine(translation, quat), 
                ReferenceType.Absolute,  # ReferenceType.Relative,
                relative_dynamics_factor=RelativeDynamicsFactor(velocity=0.01, acceleration=0.01, jerk=0.01),
            )

            try:
                self.robot.move(motion, asynchronous=not blocking)
            except ControlException as ex:
                print(ex)
                self.robot.recover_from_errors()
        else:
            raise NotImplementedError("Invalid action_space:", action_space)

        action_info = {action_space: action[:-1], 
                        "gripper_position": action[-1]}

        return action_info


    def create_action_dict(self, action, action_space, gripper_action_space=None, robot_state=None):
        assert action_space in ["joint_position", "cartesian_velocity"]
        action_dict = {action_space: action[:-1], 
                        "gripper_position": action[-1]}
        return action_dict

    #     assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
    #     if robot_state is None:
    #         robot_state = self.get_robot_state()[0]
    #     action_dict = {"robot_state": robot_state}
    #     velocity = "velocity" in action_space

    #     if gripper_action_space is None:
    #         gripper_action_space = "velocity" if velocity else "position"
    #     assert gripper_action_space in ["velocity", "position"]
            

    #     if gripper_action_space == "veltimestamp_dict
    #         action_dict["gripper_position"] = float(np.clip(action[-1], 0, 1))
    #         gripper_delta = action_dict["gripper_position"] - robot_state["gripper_position"]
    #         gripper_velocity = self._ik_solver.gripper_delta_to_velocity(gripper_delta)
    #         action_dict["gripper_delta"] = gripper_velocity

    #     if "cartesian" in action_space:
    #         if velocity:
    #             action_dict["cartesian_velocity"] = action[:-1]
    #             cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(action[:-1])
    #             action_dict["cartesian_position"] = add_poses(
    #                 cartesian_delta, robot_state["cartesian_position"]
    #             ).tolist()
    #         else:
    #             action_dict["cartesian_position"] = action[:-1]
    #             cartesian_delta = pose_diff(action[:-1], robot_state["cartesian_position"])
    #             cartesian_velocity = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
    #             action_dict["cartesian_velocity"] = cartesian_velocity.tolist()

    #         action_dict["joint_velocity"] = self._ik_solver.cartesian_velocity_to_joint_velocity(
    #             action_dict["cartesian_velocity"], robot_state=robot_state
    #         ).tolist()
    #         joint_delta = self._ik_solver.joint_velocity_to_delta(action_dict["joint_velocity"])
    #         action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()

    #     if "joint" in action_space:
    #         # NOTE: Joint to Cartesian has undefined dynamics due to IK
    #         if velocity:
    #             action_dict["joint_velocity"] = action[:-1]
    #             joint_delta = self._ik_solver.joint_velocity_to_delta(action[:-1])
    #             action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
    #         else:
    #             action_dict["joint_position"] = action[:-1]
    #             joint_delta = np.array(action[:-1]) - np.array(robot_state["joint_positions"])
    #             joint_velocity = self._ik_solver.joint_delta_to_velocity(joint_delta)
    #             action_dict["joint_velocity"] = joint_velocity.tolist()

    #     return action_dict

    # def read_cameras(self):
    #     return self.camera_reader.read_cameras()


    # def get_robot_state(self):

    #     # # TODO: Integrate From DROID

    #     # robot_state = self._robot.get_robot_state()
    #     # gripper_position = self.get_gripper_position()
    #     # pos, quat = self._robot.robot_model.forward_kinematics(torch.Tensor(robot_state.joint_positions))
    #     # cartesian_position = pos.tolist() + quat_to_euler(quat.numpy()).tolist()

    #     # state_dict = {
    #     #     "cartesian_position": cartesian_position,
    #     #     "gripper_position": gripper_position,
    #     #     "joint_positions": list(robot_state.joint_positions),
    #     #     "joint_velocities": list(robot_state.joint_velocities),
    #     #     "joint_torques_computed": list(robot_state.joint_torques_computed),
    #     #     "prev_joint_torques_computed": list(robot_state.prev_joint_torques_computed),
    #     #     "prev_joint_torques_computed_safened": list(robot_state.prev_joint_torques_computed_safened),
    #     #     "motor_torques_measured": list(robot_state.motor_torques_measured),
    #     #     "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
    #     #     "prev_command_successful": robot_state.prev_command_successful,
    #     # }

    #     # timestamp_dict = {
    #     #     "robot_timestamp_seconds": robot_state.timestamp.seconds,
    #     #     "robot_timestamp_nanos": robot_state.timestamp.nanos,
    #     # }

    #     # return state_dict, timestamp_dict

    #     state = self.robot.state

    #     # Get the robot's cartesian state
    #     robot = self.robot
    #     cartesian_state = robot.current_cartesian_state
    #     robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
    #     ee_pose = robot_pose.end_effector_pose
    #     # elbow_pos = robot_pose.elbow_position
    #     robot_velocity = cartesian_state.velocity  # Contains end-effector twist and elbow velocity
    #     ee_twist = robot_velocity.end_effector_twist
    #     elbow_vel = robot_velocity.elbow_velocity

    #     # Get the robot's joint state
    #     joint_state = robot.current_joint_state
    #     joint_pos = joint_state.position
    #     joint_vel = joint_state.velocity

    #     return {
    #         "cartesian_position": np.concatenate([ee_pose.translation, quat_to_euler(ee_pose.quaternion)]).astype(np.float64),,
    #         "gripper_position": self.gripper.motor.current_position,
            
    #         "joint_positions": state.q,
    #         "joint_velocities": state.dq,
            
    #         "joint_torques_computed": list(robot_state.joint_torques_computed),
    #         "prev_joint_torques_computed": list(robot_state.prev_joint_torques_computed),
    #         "prev_joint_torques_computed_safened": list(robot_state.prev_joint_torques_computed_safened),
    #         "motor_torques_measured": list(robot_state.motor_torques_measured),
    #         "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
    #         "prev_command_successful": robot_state.prev_command_successful,
    #     }

    #     return {
    #         "cartesian_position": np.concatenate([ee_pose.translation, quat_to_euler(ee_pose.quaternion)]).astype(np.float64),
    #         "gripper_position": 0.0, #self.gripper.motor.current_position, #self.gripper.width,
            
    #         "joint_positions": state.q,
    #         "joint_velocities": state.dq,
            
    #         "cartesian_velocity": np.concatenate([ee_twist.linear, ee_twist.angular]).astype(np.float64),

    #         # "robot_timestamp_seconds": robot_state.timestamp.seconds,
    #         # "robot_timestamp_nanos": robot_state.timestamp.nanos,
    #     }

    def get_state(self):
        read_start = time_ms()
        # state_dict, timestamp_dict = self.get_state()

        state = self.robot.state

        # Get the robot's cartesian state
        robot = self.robot
        ee_pose = robot.current_pose.end_effector_pose
        cartesian_state = robot.current_cartesian_state
        robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
        # ee_pose = robot_pose.end_effector_pose
        # elbow_pos = robot_pose.elbow_position
        robot_velocity = cartesian_state.velocity  # Contains end-effector twist and elbow velocity
        ee_twist = robot_velocity.end_effector_twist
        elbow_vel = robot_velocity.elbow_velocity

        # Get the robot's joint state
        joint_state = robot.current_joint_state
        joint_pos = joint_state.position
        joint_vel = joint_state.velocity

        state_dict = {
            "cartesian_position": np.concatenate([ee_pose.translation, quat_to_euler(ee_pose.quaternion)]).astype(np.float64),
            "cartesian_velocity": np.concatenate([ee_twist.linear, ee_twist.angular]).astype(np.float64),

            # TODO: change back
            "gripper_position": 0.0, # self.gripper.current_position_rel, #self.gripper.width,
            
            "joint_positions": state.q,
            "joint_velocities": state.dq,
            
            # "

            # "robot_timestamp_seconds": robot_state.timestamp.seconds,
            # "robot_timestamp_nanos": robot_state.timestamp.nanos,
        }

        # FROM DROID
        # robot_state = self._robot.get_robot_state()
        # gripper_position = self.get_gripper_position()
        # pos, quat = self._robot.robot_model.forward_kinematics(torch.Tensor(robot_state.joint_positions))
        # cartesian_position = pos.tolist() + quat_to_euler(quat.numpy()).tolist()

        # state_dict = {
        #     "cartesian_position": caenvrtesian_position,
        #     "gripper_position": gripper_position,
        #     "joint_positions": list(robot_state.joint_positions),
        #     "joint_velocities": list(robot_state.joint_velocities),
        #     "joint_torques_computed": list(robot_state.joint_torques_computed),
        #     "prev_joint_torques_computed": list(robot_state.prev_joint_torques_computed),
        #     "prev_joint_torques_computed_safened": list(robot_state.prev_joint_torques_computed_safened),
        #     "motor_torques_measured": list(robot_state.motor_torques_measured),
        #     "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
        #     "prev_command_successful": robot_state.prev_command_successful,
        # }

        # timestamp_dict = {
        #     "robot_timestamp_seconds": robot_state.timestamp.seconds,
        #     "robot_timestamp_nanos": robot_state.timestamp.nanos,
        # }

        timestamp_dict = {
            "read_start": read_start,
            "read_end": time_ms(),
        }
        return state_dict, timestamp_dict

    # def get_camera_extrinsics(self, state_dict):
    #     # Adjust gripper camere by current pose
    #     extrinsics = deepcopy(self.calibration_dict)
    #     for cam_id in self.calibration_dict:
    #         if gripper_cam_id not in cam_id:
    #             continue
    #         gripper_pose = state_dict["cartesian_position"]
    #         extrinsics[cam_id + "_gripper_offset"] = extrinsics[cam_id]
    #         extrinsics[cam_id] = change_pose_frame(extrinsics[cam_id], gripper_pose)
    #     return extrinsics

    def get_observation(self):
        obs_dict = {"timestamp": {}}

        # Robot State #
        state_dict, timestamp_dict = self.get_state()
        obs_dict["robot_state"] = state_dict
        obs_dict["timestamp"]["robot_state"] = timestamp_dict

        # Camera Readings #
        # start = time.time()
        #camera_obs, camera_timestamp = self.read_cameras()
        # print(f"Read Cameras: {time.time() - start}")
        #obs_dict.update(camera_obs)
        #obs_dict["timestamp"]["cameras"] = camera_timestamp

        # Camera Info #
        # obs_dict["camera_type"] = deepcopy(self.camera_type_dict)
        # extrinsics = self.get_camera_extrinsics(state_dict)
        # obs_dict["camera_extrinsics"] = extrinsics

        # intrinsics = {}
        
        # for cam in self.camera_reader.camera_dict.values():
        #     cam_intr_info = cam.get_intrinsics()
        #     for (full_cam_id, info) in cam_intr_info.items():
        #         intrinsics[full_cam_id] = info["cameraMatrix"]
        # obs_dict["camera_intrinsics"] = intrinsics

        return obs_dict
