import time

import numpy as np

from scipy.spatial.transform import Rotation
try:
    from .oculus_reader import OculusReader
    from .subprocess_utils import run_threaded_command
    from .transformations import add_angles, add_quats, euler_to_quat, quat_diff, quat_to_euler, rmat_to_quat
except:
    from oculus_reader import OculusReader
    from subprocess_utils import run_threaded_command
    from transformations import add_angles, euler_to_quat, quat_diff, quat_to_euler, rmat_to_quat

def vec_to_reorder_mat(vec):
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


class VRPolicy:
    def __init__(
        self,
        right_controller: bool = True,
        max_lin_vel: float = 0.5,
        max_rot_vel: float = 1.0,
        max_gripper_vel: float = 1.0,
        spatial_coeff: float = 1.0,
        pos_action_gain: float = 1.0,
        rot_action_gain: float = 1.0,
        gripper_action_gain: float = 3.0,
        rmat_reorder: list = [-2, -1, -3, 4],
    ):
        self.oculus_reader = OculusReader()
        self.vr_to_global_mat = np.eye(4)
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.max_gripper_vel = max_gripper_vel
        self.spatial_coeff = spatial_coeff
        self.pos_action_gain = pos_action_gain
        self.rot_action_gain = rot_action_gain
        self.gripper_action_gain = gripper_action_gain
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder)
        self.controller_id = "r" if right_controller else "l"
        self.extended_controller_id = "right" if right_controller else "left"
        self.reset_orientation = True
        self.reset_state()

        # Start State Listening Thread #
        self.is_running = True
        self.thread = run_threaded_command(self._update_internal_state)

    def reset_state(self):
        self._reader_state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled": False,
            "controller_on": True,
        }
        self.update_sensor = True
        self.reset_origin = True
        self.reset_to_robot = True
        self.reset_to_init_robot = False
        self.init = True
        self.robot_init = None
        self.robot_origin = None
        self.vr_origin = None
        self.vr_state = None
        self.vr_last_state = None
        self.last_target = None

    def _update_internal_state(self, num_wait_sec=5, hz=50):
        """
        Updating internal state continuously.
        If button `G`is pressed, we allow control.
        If button `J` is pressed, we reset the current controller orientation to the current robot rotation

        """
        last_read_time = time.time()
        while self.is_running:
            # Regulate Read Frequency
            time.sleep(1 / hz)

            # Read Controller
            time_since_read = time.time() - last_read_time
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
            self._reader_state["controller_on"] = time_since_read < num_wait_sec
            if poses == {}:
                continue

            # Determine Control Pipeline
            toggled = self._reader_state["movement_enabled"] != buttons[self.controller_id.upper() + "G"]
            self.update_sensor = self.update_sensor or buttons[self.controller_id.upper() + "G"]
            self.reset_orientation = self.reset_orientation or buttons[self.controller_id.upper() + "J"]
            # TODO: what are reset_to_robot and reset_to_init_robot
            self.reset_to_robot = self.reset_to_robot or buttons[self.extended_controller_id + "JS"][1] > 0.5
            self.reset_to_init_robot = self.reset_to_init_robot or buttons[self.extended_controller_id + "JS"][1] < -0.5
            self.reset_origin = self.reset_origin or toggled

            # Save Info
            self._reader_state["poses"] = poses
            self._reader_state["buttons"] = buttons
            self._reader_state["movement_enabled"] = buttons[self.controller_id.upper() + "G"]
            self._reader_state["controller_on"] = True
            last_read_time = time.time()

            # We stop to update the orientation if button `J` was pressed or we aim to teleoperate
            stop_updating = self._reader_state["buttons"][self.controller_id.upper() + "J"] or self._reader_state["movement_enabled"]

            # We reset the orientation after pressing button `J` or if self.reset_state() is called
            if self.reset_orientation:
                print(time.time(), self.extended_controller_id, "reset orientation", flush=True)
                rot_mat = np.asarray(self._reader_state["poses"][self.controller_id])
                if stop_updating:
                    self.reset_orientation = False
                # try to invert the rotation matrix, if not possible, then just use the identity matrix
                try:
                    rot_mat = np.linalg.inv(rot_mat)
                except:
                    print(time.time(), self.extended_controller_id, f"exception for rot mat: {rot_mat}", flush=True)
                    rot_mat = np.eye(4)
                    self.reset_orientation = True
                self.vr_to_global_mat = rot_mat

    def _process_reading(self):
        rot_mat = np.asarray(self._reader_state["poses"][self.controller_id])
        rot_mat = self.global_to_env_mat @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3]
        vr_quat = rmat_to_quat(rot_mat[:3, :3] @ Rotation.from_euler("xyz", [0, 0, -np.pi / 2]).as_matrix())
        # vr_quat = rmat_to_quat(rot_mat[:3, :3])
        vr_gripper = self._reader_state["buttons"]["rightTrig" if self.controller_id == "r" else "leftTrig"][0]

        # copy state to last_state
        if self.vr_state != None:
            self.vr_last_state = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}

        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}

    def _vr_state_standstill(self):
        if self._reader_state == None or self.vr_last_state == None:
            return True
        if (self.vr_state["pos"] - self.vr_last_state["pos"]).all() == 0 and (self.vr_state["quat"] - self.vr_last_state["quat"]).all() == 0:
            return True
        return False

    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Scales down the linear and angular magnitudes of the action"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.linalg.norm(gripper_vel)
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        if gripper_vel_norm > self.max_gripper_vel:
            gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
        return lin_vel, rot_vel, gripper_vel

    def _calculate_action(self, robot_state_dict):
        """
        Derive robot target position from
        """
        # Read Sensor #
        self._process_reading()

        # Read Observation
        robot_pos = np.array(robot_state_dict["cartesian_position"])
        robot_quat = np.array(robot_state_dict["cartesian_rotation"])
        robot_euler = quat_to_euler(robot_quat)
        robot_gripper = robot_state_dict["gripper_position"]

        if self.init:
            print(time.time(), self.extended_controller_id, "init", flush=True)
            self.robot_init = {"pos": robot_pos, "quat": robot_quat}
            self.robot_origin = {"pos": robot_pos, "quat": robot_quat}
            self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
            self.last_target = {"pos": robot_pos, "quat": robot_quat}
            self.init = False

        # Reset Origin On Release
        if self.reset_origin:
            if self._reader_state["movement_enabled"]:
                print(time.time(), self.extended_controller_id, "start movement", flush=True)
                self.robot_origin = {"pos": robot_pos, "quat": robot_quat} # self.last_target
                self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
            else:
                print(time.time(), self.extended_controller_id, "stop movement", flush=True)
            self.reset_origin = False

        if self._vr_state_standstill():
            if self.vr_state != None and self.vr_last_state != None:
                print(time.time(), self.extended_controller_id, "no action detected, meta disconect/calibration?", self.vr_state["pos"], "vs", self.vr_last_state["pos"], flush=True)

        # Calculate Positional Action
        robot_pos_offset = robot_pos - self.robot_origin["pos"]
        # print(f'robot_pos_offset: {robot_pos_offset}')
        # print(f'')
        vr_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
        # print(f'vr_pos_offset: {vr_pos_offset}')

        # Calculate Euler Action #
        # TODO: remove robot_quat_offset, vr_quat_offset
        robot_quat_offset = quat_diff(robot_quat, self.robot_origin["quat"])
        vr_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
        # target_quat = add_quats(self.robot_origin["quat"], self.rot_action_gain * quat_diff(self.vr_state["quat"], self.vr_origin["quat"]))
        offset_q = Rotation.from_quat(robot_quat_offset)
        offset_q2 = Rotation.from_quat(vr_quat_offset)
        # print(f'offset_q (deg): {offset_q.as_euler("xyz", degrees=True)}')
        # print(f'offset_q2 (deg): {offset_q2.as_euler("xyz", degrees=True)}')
        # Calculate VR Delta in Global Frame: R_delta = R_current * R_origin^-1
        R_vr_curr = Rotation.from_quat(self.vr_state["quat"])
        R_vr_orig = Rotation.from_quat(self.vr_origin["quat"])
        R_delta = R_vr_curr * R_vr_orig.inv()

        # Scale Rotation (Angle-Axis scaling)
        if self.rot_action_gain != 1.0:
            rotvec = R_delta.as_rotvec()
            R_delta = Rotation.from_rotvec(rotvec * self.rot_action_gain)

        # Apply Global Delta to Robot Origin: R_target = R_delta * R_robot_origin
        R_robot_orig = Rotation.from_quat(self.robot_origin["quat"])
        target_quat = (R_delta * R_robot_orig).as_quat()

        target_pos = self.robot_origin["pos"] + self.pos_action_gain * vr_pos_offset
        # target_quat = add_quats(self.robot_origin["quat"], self.rot_action_gain * quat_diff(self.vr_state["quat"], self.vr_origin["quat"]))

        target_gripper = self.vr_state["gripper"]

        if not self._reader_state["movement_enabled"]:
            target_pos = self.last_target["pos"]
            target_quat = self.last_target["quat"]

        if self.reset_to_robot:
            print(time.time(), self.extended_controller_id, "reset target to robot", flush=True)
            self.reset_to_robot = False
            target_pos = robot_pos
            target_quat = robot_quat

        if self.reset_to_init_robot:
            print(time.time(), self.extended_controller_id, "reset target to init robot", flush=True)
            self.reset_to_init_robot = False
            target_pos = self.robot_init["pos"]
            target_quat = self.robot_init["quat"]

        # info
        info_dict = {
            "movement_enabled": self._reader_state["movement_enabled"],
            
            "vr_raw_pos": self.vr_state["pos"],
            "vr_raw_quat": self.vr_state["quat"],
            "vr_origin_pos": self.vr_origin["pos"],
            "vr_origin_quat": self.vr_origin["quat"],
            "vr_pos": vr_pos_offset,
            "vr_quat": vr_quat_offset,

            "robot_raw_pos": robot_pos,
            "robot_raw_quat": robot_quat,
            "robot_origin_pos": self.robot_origin["pos"],
            "robot_origin_quat": self.robot_origin["quat"],
            "robot_pos": robot_pos_offset,
            "robot_quat": robot_quat_offset,

            "robot_target_pos": target_pos,
            "robot_target_quat": target_quat,     
            }
        # Return #
        return np.concatenate([target_pos, target_quat]), target_gripper, info_dict

    def get_info(self):
        return {
            "success": self._reader_state["buttons"]["A"] if self.controller_id == 'r' else self._reader_state["buttons"]["X"],
            "failure": self._reader_state["buttons"]["B"] if self.controller_id == 'r' else self._reader_state["buttons"]["Y"],
            "movement_enabled": self._reader_state["movement_enabled"],
            "controller_on": self._reader_state["controller_on"],
        }

    def forward(self, obs_dict):
        if self._reader_state["poses"] == {}:
            return np.zeros(6), np.zeros(1), {}
        return self._calculate_action(obs_dict)

    def close(self):
        self.is_running = False
        self.thread.join()
