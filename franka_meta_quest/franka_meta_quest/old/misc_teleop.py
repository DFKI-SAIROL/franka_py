import time
from collections import defaultdict
from copy import deepcopy

import cv2
import numpy as np
from PIL import Image

from droid.calibration.calibration_utils import *
#from droid.camera_utils.info import camera_type_to_string_dict
#from droid.camera_utils.wrappers.recorded_multi_camera_wrapper import RecordedMultiCameraWrapper
from droid.misc.parameters import *
from droid.misc.time import time_ms
from droid.misc.transformations import change_pose_frame
from droid.trajectory_utils.trajectory_reader import TrajectoryReader
from droid.trajectory_utils.trajectory_writer import TrajectoryWriter

#from droid.evaluation.atacom.franky_constraints import OrientedBoundingBox

from franky import JointWaypointMotion, JointWaypoint

try:
    from franky import CartesianVelocityStopMotion
except:
    from franky import CartesianVelocitiesStopMotion as CartesianVelocityStopMotion


# bbox = OrientedBoundingBox(
#         center=np.array([400., 200., 110]),
#         extent=np.array([160, 70, 220]),
#         rotation=np.eye(3),
#     )

        # u_[3:6] = 0.
        
        # error = (self._q_init - q) * self.error_joint_weights
        # action += (np.eye(7) - np.linalg.pinv(jac) @ jac) @ error
        # action = action / np.maximum(1.0, (np.abs(action) / self._vel_limit[1]).max())
        
def relative_l2_difference(original_action, safe_action):
    """
    Computes the relative L2 norm difference between the original policy action
    and the morphed (safe) action.
    
    Parameters:
        original_action (np.array): The original policy action.
        safe_action (np.array): The safe/morphed version of the action.
        
    Returns:
        float: The relative L2 norm difference.
    """
    norm_diff = np.linalg.norm(original_action - safe_action)
    norm_original = np.linalg.norm(original_action)
    
    # Avoid division by zero
    if norm_original == 0:
        return np.inf if norm_diff != 0 else 0.0
    
    return norm_diff / norm_original


def collect_trajectory(
    env,
    controller=None,
    policy=None,
    atacom_agent=None,
    horizon=None,
    save_filepath=None,
    metadata=None,
    wait_for_controller=False,
    obs_pointer=None,
    save_images=False,
    recording_folderpath=False,
    randomize_reset=False,
    reset_robot=True,
):
    """
    Collects a robot trajectory.
    - If policy is None, actions will come from the controller
    - If a horizon is given, we will step the environment accordingly
    - Otherwise, we will end the trajectory when the controller tells us to
    - If you need a pointer to the current observation, pass a dictionary in for obs_pointer
    """
    time.sleep(0.5)
    # Check Parameters #
    assert (controller is not None) or (policy is not None)
    assert (controller is not None) or (horizon is not None)
    if wait_for_controller:
        assert controller is not None
    if obs_pointer is not None:
        assert isinstance(obs_pointer, dict)
    if save_images:
        assert save_filepath is not None

    # Reset States #
    if controller is not None:
        controller.reset_state()
    # env.camera_reader.set_trajectory_mode()

    # Prepare Data Writers If Necesary #
    if save_filepath:
        traj_writer = TrajectoryWriter(save_filepath, metadata=metadata, save_images=save_images)
    # if recording_folderpath:
    #     env.camera_reader.start_recording(recording_folderpath)

    # Prepare For Trajectory #
    num_steps = 0
    if reset_robot:
        env.reset(randomize=randomize_reset)
    
    last_update = None

    # Begin! #
    while True:
        # Collect Miscellaneous Info #
        controller_info = {} if (controller is None) else controller.get_info()
        skip_action = wait_for_controller and (not controller_info["movement_enabled"])
        control_timestamps = {"step_start": time_ms()}

        # Get Observation #
        obs = env.get_observation()
        if obs_pointer is not None:
            obs_pointer.update(obs)
        obs["controller_info"] = controller_info
        obs["timestamp"]["skip_action"] = skip_action

        # Get Action #
        control_timestamps["policy_start"] = time_ms()
        if policy is None:
            action, controller_action_info = controller.forward(obs, include_info=True)
        else:
            action = policy.forward(obs)
            controller_action_info = {}

        # Regularize Control Frequency #
        control_timestamps["sleep_start"] = time_ms()
        #comp_time = time_ms() - control_timestamps["step_start"]
        now = time.time()
        if last_update is not None:
            next_update = last_update + 1 / env.control_hz
            residual_time = next_update - now
            # print(residual_time)
            if residual_time < 0:
                exceeded_time_in_ms = -residual_time * 1000
                print(f"WARNING: Control interval exceeded by {exceeded_time_in_ms:0.2f}ms")
            # print(f"Residual time: {next_update - now}")
            sleep_time = max(0, residual_time)
            # print("sleep time", sleep_time)
            time.sleep(sleep_time)
        last_update = now
        #sleep_left = (1 / env.control_hz) - (comp_time / 1000)
        # if sleep_left > 0:
        #     time.sleep(sleep_left)

        # Moniter Control Frequency #
        # moniter_control_frequency = True
        # if moniter_control_frequency:
        # 	print('Sleep Left: ', sleep_left)
        # 	print('Feasible Hz: ', (1000 / comp_time))

        # Step Environment #
        control_timestamps["control_start"] = time_ms()
        if skip_action:
            action_info = env.create_action_dict(np.zeros_like(action), env.action_space)
        else:
            #action_info = env.step(np.concatenate([controller_action_info["target_cartesian_position"], np.zeros(1)]))
            # print("Robot control with: ", action)
            
            # if atacom_agent:
            #     obs["bbox"] = bbox
            #     action, original_action = atacom_agent.safe_action(obs, action, controller.robot_origin)
                
            #     relative_l2 = relative_l2_difference(original_action, action)
            #     print(f"relative l2 action difference : {relative_l2}")
            
            action = np.clip(action, -1, 1)
            action_info = env.step(action)
        action_info.update(controller_action_info)

        # Save Data #
        control_timestamps["step_end"] = time_ms()
        obs["timestamp"]["control"] = control_timestamps
        timestep = {"observation": obs, "action": action_info}
        if save_filepath:
            traj_writer.write_timestep(timestep)

        # Check Termination #
        num_steps += 1
        if horizon is not None:
            end_traj = horizon == num_steps
        else:
            end_traj = controller_info["success"] or controller_info["failure"]

        # Close Files And Return #
        if end_traj:
            try:
                env.robot.move(CartesianVelocityStopMotion())
            except Exception as e:
                print("Warning: ", e)
            # if recording_folderpath:
            #     env.camera_reader.stop_recording()
            if save_filepath:
                traj_writer.close(metadata=controller_info)
                print("Saved trajectory")
            return controller_info
        
        # pprint({k: v - control_timestamps["step_start"] for k, v in control_timestamps.items()})


def calibrate_camera(
    env,
    camera_id,
    controller,
    step_size=0.01,
    pause_time=0.5,
    image_freq=10,
    obs_pointer=None,
    wait_for_controller=False,
    reset_robot= None, #"default",  # Available modes: 'default', 'controller'
    camera_side = 'left', # TODO: temporary fix for camera naming
):
    """Returns true if calibration was successful, otherwise returns False
    3rd Person Calibration Instructions: Press A when board in aligned with the camera from 1 foot away.
    Hand Calibration Instructions: Press A when the hand camera is aligned with the board from 1 foot away."""

    # if reset_robot is not None:
    #     if reset_robot == "default" or True:
    #         # Reset robot to predefined joint positions

    #         env.robot.recover_from_errors()

    #         calibration_joint_position = {
    #             '52312672': [ 0.74647443, -0.27357481, -0.08623657, -2.17585743,  0.18639579, 1.83816842,  2.06313621],  # left
    #             # '52312672': [ 0.75373021, -0.35875266,  0.15028619, -2.40859328,  0.34531727, 2.04513143,  2.09132382],  # left
    #             '56332516': [-2.49785996, -0.79144782,  1.23967349, -2.02475351,  0.47514176, 1.78064695, -1.08747445], # right
    #             '53951915': [-0.44356839,  0.13602044, -0.0906438 , -1.99720446,  0.03831328, 2.74134885, -2.23846781], # front
    #         }[camera_id]

    #         env.reset()

    #         motion = JointWaypointMotion([JointWaypoint(calibration_joint_position)], relative_dynamics_factor=0.1)
    #         env.robot.move(motion)

    #         # time.sleep(5)
    #         # print("GOII!")
    #         # positions = [env.robot.current_joint_positions]
    #         # try:
    #         #     while True:
    #         #         time.sleep(3)
    #         #         positions.append(env.robot.current_joint_positions)
    #         #         print(positions[-1])
    #         # except:
    #         #     from pprint import pprint
    #         #     pprint(positions)

    #         # traj = JointWaypointMotion([JointWaypoint(p) for p in np.stack(positions)], relative_dynamics_factor=0.2)
                    
    #         # env.robot.move(traj)

    #         print("Robot reset to calibration position")

    #     elif reset_robot == "controller":
    #         raise NotImplementedError
    #         # Reset robot using the oculus controller
    #     else:
    #         print("reset_robot must be 'default' or 'controller'")
    # else:
    #     print("WARNING: Manual resetting")
    #     input("Press Enter to confirm, that the robot is reset correctly")



    if obs_pointer is not None:
        assert isinstance(obs_pointer, dict)

    # Get Camera + Set Calibration Mode #
    camera = env.camera_reader.get_camera(camera_id)
    env.camera_reader.set_calibration_mode(camera_id)
    assert pause_time > (camera.latency / 1000)

    # # Select Proper Calibration Procedure #
    hand_camera = False  # camera.serial_number == hand_camera_id
    intrinsics_dict = camera.get_intrinsics()
    if hand_camera:
        calibrator = HandCameraCalibrator(intrinsics_dict)
    else:
        calibrator = ThirdPersonCameraCalibrator(intrinsics_dict)

    # while True:
    #     # Collect Controller Info #
    #     controller_info = controller.get_info()
    #     start_time = time.time()

    #     # Get Observation #
    #     state, _ = env.get_state()
    #     cam_obs, _ = env.read_cameras()

    #     for full_cam_id in cam_obs["image"]:
    #         if f"{camera_id}_{camera_side}" not in full_cam_id:
    #             continue
    #         cam_obs["image"][full_cam_id] = calibrator.augment_image(full_cam_id, cam_obs["image"][full_cam_id])
    #     if obs_pointer is not None:
    #         obs_pointer.update(cam_obs)

    #     # Get Action #
    #     action = controller.forward({"robot_state": state})
    #     action[-1] = 0  # Keep gripper open

    #     # Regularize Control Frequency #
    #     comp_time = time.time() - start_time
    #     sleep_left = (1 / env.control_hz) - comp_time
    #     if sleep_left > 0:
    #         time.sleep(sleep_left)

    #     # Step Environment #
    #     skip_step = wait_for_controller and (not controller_info["movement_enabled"])
    #     if not skip_step:
    #         env.step(action)

    #     # Check Termination #
    #     start_calibration = controller_info["success"]
    #     end_calibration = controller_info["failure"]

    #     # Close Files And Return #
    #     if start_calibration:
    #         break
    #     if end_calibration:
    #         return False

    # Collect Data #
    state, _ = env.get_state()
    time.time()
    pose_origin = state["cartesian_position"]
    i = 0

    while True:
        # Check For Termination #
        # controller_info = controller.get_info()
        # if controller_info["failure"]:
        #     return False

        # Start #
        start_time = time.time()
        take_picture = True #(i % image_freq) == 0

        # Collect Observations #
        if take_picture:
            time.sleep(pause_time)
        state, _ = env.get_state()
        cam_obs, _ = env.read_cameras()

        # Add Sample + Augment Images #
        for full_cam_id in cam_obs["image"]:
            if f"{camera_id}_{camera_side}" not in full_cam_id:
                continue
            if take_picture:
                img = deepcopy(cam_obs["image"][full_cam_id])
                pose = state["cartesian_position"].copy()
                added_image = calibrator.add_sample(full_cam_id, img, pose)
                print("IMAGE ADDED", added_image)
            cam_obs["image"][full_cam_id] = calibrator.augment_image(full_cam_id, cam_obs["image"][full_cam_id])
        
        print("SLEEPING", i)
        time.sleep(2)

        # Update Obs Pointer #
        if obs_pointer is not None:
            obs_pointer.update(cam_obs)

        # Move To Desired Next Pose #
        calib_pose = calibration_traj(i * step_size, hand_camera=hand_camera, angle_scale=0.2)
        desired_pose = change_pose_frame(calib_pose, pose_origin)
        action = np.concatenate([desired_pose, [0]])
        # env.update_robot(action, action_space="cartesian_position", blocking=True)


        # Regularize Control Frequency #
        comp_time = time.time() - start_time
        sleep_left = (1 / env.control_hz) - comp_time
        if sleep_left > 0:
            time.sleep(sleep_left)

        # Check If Cycle Complete #
        cycle_complete = (i * step_size) >= 0.7
        if cycle_complete:
        #     try:
        #         env.robot.join_motion()
        #     except Exception as e:
        #         time.sleep(1)
        #         env.robot.join_motion()
            break
        i += 1  

    # SAVE INTO A JSON
    for full_cam_id in cam_obs["image"]:
        if f"{camera_id}_{camera_side}" not in full_cam_id:
            continue
        success = calibrator.is_calibration_accurate(full_cam_id)
        if not success:
            print("calibration failed")
            return False
        transformation = calibrator.calibrate(full_cam_id)
        update_calibration_info(full_cam_id, transformation)
        print("calibration successfull")

    return True


def replay_trajectory(
    env, filepath=None, 
    assert_replayable_keys=["cartesian_position", "gripper_position", "joint_positions"], 
    reset_robot=True, 
    randomize_reset=False,
):
    if reset_robot:
        env.reset(randomize=randomize_reset)

    print("WARNING: STATE 'CLOSENESS' FOR REPLAYABILITY HAS NOT BEEN CALIBRATED")
    # gripper_key = "gripper_velocity" if "velocity" in env.action_space else "gripper_position"

    gripper_key = "gripper_position"

    # Prepare Trajectory Reader #
    traj_reader = TrajectoryReader(filepath, read_images=False)
    horizon = traj_reader.length()

    last_update = None
    for i in range(horizon):
        # Get HDF5 Data #
        timestep = traj_reader.read_timestep()

        timestep["action"]['joint_velocity'] = timestep["observation"]['robot_state']['joint_velocities']
        timestep["action"]['cartesian_velocity'] = timestep["observation"]['robot_state']['cartesian_velocity']

        # Move To Initial Position #
        if i == 0:
            init_joint_position = timestep["observation"]["robot_state"]["joint_positions"]
            init_gripper_position = timestep["observation"]["robot_state"]["gripper_position"]
            action = np.concatenate([init_joint_position, [init_gripper_position]])
            env.update_robot(action, action_space="joint_position", blocking=True)

        # Assert Replayability - robot needs to be at the initial position of the trajectory to be replayed
        robot_state = env.get_state()[0]
        for key in assert_replayable_keys:
            desired = timestep['observation']['robot_state'][key]
            current = robot_state[key]
            print(desired - current)
            # assert np.allclose(desired, current)

        # # Regularize Control Frequency #
        # time.sleep(1 / env.control_hz)

        # Regularize Control Frequency #
        now = time.time()
        if last_update is not None:
            next_update = last_update + 1 / env.control_hz
            sleep_time = max(0, next_update - now)
            # print("sleep time", sleep_time)
            time.sleep(sleep_time)
        last_update = now

        # Get Action In Desired Action Space #
        assert env.action_space in timestep["action"], \
        "Selected action space does not align with recorded action space"
        arm_action = timestep["action"][env.action_space]
        gripper_action = timestep["action"][gripper_key]
        action = np.concatenate([arm_action, [gripper_action]])
        controller_info = timestep["observation"]["controller_info"]
        movement_enabled = controller_info.get("movement_enabled", True)

        # Follow Trajectory #
        if movement_enabled:
            env.step(action)


def load_trajectory(
    filepath=None,
    read_cameras=True,
    recording_folderpath=None,
    camera_kwargs={},
    remove_skipped_steps=False,
    num_samples_per_traj=None,
    num_samples_per_traj_coeff=1.5,
):
    read_hdf5_images = read_cameras and (recording_folderpath is None)
    read_recording_folderpath = read_cameras and (recording_folderpath is not None)

    traj_reader = TrajectoryReader(filepath, read_images=read_hdf5_images)
    if read_recording_folderpath:
        camera_reader = RecordedMultiCameraWrapper(recording_folderpath, camera_kwargs)

    horizon = traj_reader.length()
    timestep_list = []

    # Choose Timesteps To Save #
    if num_samples_per_traj:
        num_to_save = num_samples_per_traj
        if remove_skipped_steps:
            num_to_save = int(num_to_save * num_samples_per_traj_coeff)
        max_size = min(num_to_save, horizon)
        indices_to_save = np.sort(np.random.choice(horizon, size=max_size, replace=False))
    else:
        indices_to_save = np.arange(horizon)

    # Iterate Over Trajectory #
    for i in indices_to_save:
        # Get HDF5 Data #
        timestep = traj_reader.read_timestep(index=i)

        # TODO Add correctly to trajectory recording!!
        #timestep["observation"]["camera_type"] = {'192.168.201.212:40004': "external_cam",
        #                                          '192.168.201.212:40000': "gripper_cam",
        #                                        }

        # If Applicable, Get Recorded Data #
        if read_recording_folderpath:
            timestamp_dict = timestep["observation"]["timestamp"]["cameras"]
            #camera_type_dict = timestep["observation"]["camera_type"]
            # camera_type_dict = {
            #     k: camera_type_to_string_dict[v] for k, v in timestep["observation"]["camera_type"].items()
            # }
            camera_obs = camera_reader.read_cameras(
                index=i, camera_type_dict={}, timestamp_dict=timestamp_dict
            )
            camera_failed = camera_obs is None

            # Add Data To Timestep If Successful #
            if camera_failed:
                break
            else:
                timestep["observation"].update(camera_obs)

        # Filter Steps #
        step_skipped = not timestep["observation"]["controller_info"].get("movement_enabled", True)
        delete_skipped_step = step_skipped and remove_skipped_steps

        # Save Filtered Timesteps #
        if delete_skipped_step:
            del timestep
        else:
            timestep_list.append(timestep)

    # Remove Extra Transitions #
    timestep_list = np.array(timestep_list)
    if (num_samples_per_traj is not None) and (len(timestep_list) > num_samples_per_traj):
        ind_to_keep = np.random.choice(len(timestep_list), size=num_samples_per_traj, replace=False)
        timestep_list = timestep_list[ind_to_keep]

    # Close Readers #
    traj_reader.close()
    if read_recording_folderpath:
        camera_reader.disable_cameras()

    # Return Data #
    return timestep_list


def visualize_timestep(timestep, max_width=1000, max_height=500, aspect_ratio=1.5, pause_time=15):
    # Process Image Data #
    obs = timestep["observation"]
    if "image" in obs:
        img_obs = obs["image"]
    elif "image" in obs["camera"]:
        img_obs = obs["camera"]["image"]
    else:
        raise ValueError

    camera_ids = sorted(img_obs.keys())
    sorted_image_list = []
    for cam_id in camera_ids:
        data = img_obs[cam_id]
        if type(data) == list:
            sorted_image_list.extend(data)
        else:
            sorted_image_list.append(data)

    # Get Ideal Number Of Rows #
    num_images = len(sorted_image_list)
    max_num_rows = int(num_images**0.5)
    for num_rows in range(max_num_rows, 0, -1):
        num_cols = num_images // num_rows
        if num_images % num_rows == 0:
            break

    # Get Per Image Shape #
    max_img_width, max_img_height = max_width // num_cols, max_height // num_rows
    if max_img_width > aspect_ratio * max_img_height:
        img_width, img_height = max_img_width, int(max_img_width / aspect_ratio)
    else:
        img_width, img_height = int(max_img_height * aspect_ratio), max_img_height

    # Fill Out Image Grid #
    img_grid = [[] for i in range(num_rows)]

    for i in range(len(sorted_image_list)):
        img = Image.fromarray(sorted_image_list[i])
        resized_img = img.resize((img_width, img_height), Image.Resampling.LANCZOS)
        img_grid[i % num_rows].append(np.array(resized_img))

    # Combine Images #
    for i in range(num_rows):
        img_grid[i] = np.hstack(img_grid[i])
    img_grid = np.vstack(img_grid)

    # Visualize Frame #
    cv2.imshow("Image Feed", img_grid)
    cv2.waitKey(pause_time)


def visualize_trajectory(
    filepath,
    recording_folderpath=None,
    remove_skipped_steps=False,
    camera_kwargs={},
    max_width=1000,
    max_height=500,
    aspect_ratio=1.5,
):
    traj_reader = TrajectoryReader(filepath, read_images=True)
    if recording_folderpath:
        if camera_kwargs is {}:
            camera_kwargs = defaultdict(lambda: {"image": True})
        camera_reader = RecordedMultiCameraWrapper(recording_folderpath, camera_kwargs)

    horizon = traj_reader.length()
    camera_failed = False

    for i in range(horizon):
        # Get HDF5 Data #
        timestep = traj_reader.read_timestep()

        # If Applicable, Get Recorded Data #
        if recording_folderpath:
            timestamp_dict = timestep["observation"]["timestamp"]["cameras"]
            # camera_type_dict = {
            #     k: camera_type_to_string_dict[v] for k, v in timestep["observation"]["camera_type"].items()
            # }
            camera_obs = camera_reader.read_cameras(
                index=i, 
                # camera_type_dict=camera_type_dict, 
                timestamp_dict=timestamp_dict
            )
            camera_failed = camera_obs is None

            # Add Data To Timestep #
            if not camera_failed:
                timestep["observation"].update(camera_obs)

        # Filter Steps #
        step_skipped = not timestep["observation"]["controller_info"].get("movement_enabled", True)
        delete_skipped_step = step_skipped and remove_skipped_steps
        delete_step = delete_skipped_step or camera_failed
        if delete_step:
            continue

        # Get Image Info #
        assert "image" in timestep["observation"]
        img_obs = timestep["observation"]["image"]
        camera_ids = list(img_obs.keys())
        len(camera_ids)
        camera_ids.sort()

        # Visualize Timestep #
        visualize_timestep(
            timestep, max_width=max_width, max_height=max_height, aspect_ratio=aspect_ratio, pause_time=15
        )

    # Close Readers #
    traj_reader.close()
    if recording_folderpath:
        camera_reader.disable_cameras()
