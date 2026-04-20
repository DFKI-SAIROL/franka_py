#!/usr/bin/env python3
import math
import numpy as np
import threading
import sys
import termios
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Time, Duration as ROSDuration
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from franka_custom_msgs.msg import FMQDebug, FIJKDebug
from std_srvs.srv import Trigger

try:
    from .oculus_controller import VRPolicy
    from .transformations import euler_to_quat
except:
    from oculus_controller import VRPolicy
    from transformations import euler_to_quat

class CartesianPosePublisher(Node):

    def __init__(self):
        super().__init__('meta_quest')

        self.ns = self.get_namespace()
        if self.ns == '/':
            self.ns = ''
            
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('end_effector_frame', 'rh_p12_rn_grasp_point')
        
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        # Internal state
        self.last_raw_target = None
        self.end_effector_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        
        self.base_frame = "world"
        self.end_effector_frame = "rh_p12_rn_grasp_point"

        print(self.base_frame)
        print(self.end_effector_frame)

        # fallback for bimanual prefix parsing if left default
        # if self.base_frame == 'base' and self.ns != '':
        #     self.base_frame = self.ns[1:] + '_' + self.base_frame
        if self.ns != '':
            self.end_effector_frame = self.ns[1:] + '_' + self.end_effector_frame

        self.joint_subscriber_ = self.create_subscription(JointState, self.ns + '/franka/joint_states', self.joint_state_callback, 1)
        self.gripper_subscriber_ = self.create_subscription(JointState, self.ns + '/franka_gripper/joint_states', self.gripper_state_callback, 1)
        
        self.fijk_subscriber_ = self.create_subscription(FIJKDebug, self.ns + '/fijk_debug', self.fijk_callback, 1)

        self.target_pose_publisher_ = self.create_publisher(PoseStamped, self.ns + '/target_pose', 1)
        self.gripper_publisher_ = self.create_publisher(JointTrajectory, self.ns + '/gripper/gripper_controller/joint_trajectory', 1)
        self.debug_publisher_ = self.create_publisher(FMQDebug, self.ns + '/mq_debug', 1)
        # TODO: add YAML
        self.timer = self.create_timer(1.0 / 50, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.init_transform = None

        self.current_joint_state = JointState()
        self.current_gripper_state = JointState()

        self.latest_cmd_pos = None
        self.latest_cmd_quat = None

        self.controller = VRPolicy()

        self.controller.reset_state()
        
        # Service Clients for Data Collection
        self.cli_rec = self.create_client(Trigger, '/data_collector/record_data_trigger')
        
        # State tracking for buttons
        self.last_buttons = {"A": False, "B": False, "X": False, "Y": False}

        self.get_logger().info('VRPolicy Publisher started.')

    def fijk_callback(self, msg):
        """Continuously caches the exact mathematical target the C++ node is holding."""
        self.latest_cmd_pos = np.array([msg.cmd_pose.position.x, msg.cmd_pose.position.y, msg.cmd_pose.position.z])
        self.latest_cmd_quat = np.array([
            msg.cmd_pose.orientation.x, 
            msg.cmd_pose.orientation.y, 
            msg.cmd_pose.orientation.z, 
            msg.cmd_pose.orientation.w
        ])

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def gripper_state_callback(self, msg):
        self.current_gripper_state = msg

    def call_service_async(self, client, name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {name} not available')
            return
        self.get_logger().info(f'Calling {name}...')
        req = Trigger.Request()
        future = client.call_async(req)
        # We don't block waiting for result to avoid freezing the loop

    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.end_effector_frame, rclpy.time.Time(), Duration(seconds=0.05))
            translation = np.array([transform.transform.translation.x,
                                   transform.transform.translation.y,
                                   transform.transform.translation.z])
            rotation = np.array([transform.transform.rotation.x, 
                                 transform.transform.rotation.y, 
                                 transform.transform.rotation.z, 
                                 transform.transform.rotation.w])

            return True, translation, rotation
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")
            return False, np.zeros(3), np.zeros(4)
        
    def to_ros_point(self, p):
        ros_point = Point()
        ros_point.x = p[0]
        ros_point.y = p[1]
        ros_point.z = p[2]
        return ros_point

    def to_ros_quat(self, q):
        ros_quat = Quaternion()
        ros_quat.x = q[0]
        ros_quat.y = q[1]
        ros_quat.z = q[2]
        ros_quat.w = q[3]
        return ros_quat

    def _apply_safety_shields(self, target_pos, target_quat, controller_action_info):
        movement_enabled = controller_action_info["movement_enabled"]
        vr_pos = controller_action_info["vr_pos"]

        if not movement_enabled:
            self.last_raw_target = None # Reset raw tracker on button release
            return target_pos, target_quat

        # Determine tracking delta relative to original pressing point
        if self.last_raw_target is None or self.controller.last_target is None:
            self.last_raw_target = {"pos": vr_pos.copy(), "quat": target_quat.copy()}
            return self.controller.last_target["pos"], self.controller.last_target["quat"]

        # 1. OUTLIER REJECTION: Check instant jumps on the unscaled VR tracking offsets
        raw_pos_diff = vr_pos - self.last_raw_target["pos"]
        raw_pos_dist = np.linalg.norm(raw_pos_diff)

        # Always update the raw tracker memory so the headset can recover from glitches
        self.last_raw_target = {"pos": vr_pos.copy(), "quat": target_quat.copy()}
        
        # Max speeds (per 1/15th of a second)
        MAX_POS_STEP = 0.02   # 45 cm/s robot limit
        MAX_QUAT_STEP = 0.15  # 135 deg/s robot limit
        
        # Outlier Rejection: 0.08m per VR tick = 1.2 m/s jumping. 
        if raw_pos_dist > 0.04:
            self.get_logger().warn(f"Dropped VR frame: impossible tracker jump ({raw_pos_dist:.3f}m > 0.08m)")
            # Keep robot safely exactly where it was previously commanded
            return self.controller.last_target["pos"], self.controller.last_target["quat"]

        # 2. DELTA CLAMPING: How far does the ROBOT need to move this tick?
        pos_diff = target_pos - self.controller.last_target["pos"]
        pos_dist = np.linalg.norm(pos_diff)

        # Linear Clamp (Trailing smoothly without warning spam)
        if pos_dist > MAX_POS_STEP:
            target_pos = self.controller.last_target["pos"] + (pos_diff / pos_dist) * MAX_POS_STEP

        # Slerp Clamp for quaternion
        current_quat = self.controller.last_target["quat"]
        dot = np.dot(current_quat, target_quat)
        if dot < 0.0:
            target_quat = -target_quat
            dot = -dot
        
        dot = np.clip(dot, -1.0, 1.0)
        theta_0 = np.arccos(dot)
        
        if theta_0 > MAX_QUAT_STEP:
            q3 = target_quat - current_quat * dot
            q3 = q3 / np.linalg.norm(q3)
            target_quat = current_quat * np.cos(MAX_QUAT_STEP) + q3 * np.sin(MAX_QUAT_STEP)
            target_quat = target_quat / np.linalg.norm(target_quat)

        return target_pos, target_quat

    def _handle_recording_triggers(self, controller_info):
        """Map controller buttons to ROS data collection services"""
        a_pressed = controller_info.get("success", False) # A or X
        b_pressed = controller_info.get("failure", False) # B or Y
        
        # Detect rising edge for A
        if a_pressed and not self.last_buttons["A"]: 
             self.call_service_async(self.cli_rec, 'Recording')
             
        self.last_buttons["A"] = a_pressed
        self.last_buttons["B"] = b_pressed
        
        return a_pressed, b_pressed

    def _get_current_robot_state(self):
        """Fetch current cartesian and gripper state of the real robot"""
        succ, translation, rotation = self.lookup_transform()
        if not succ:
            return False, None
            
        robot_state_dict = {
            "cartesian_position": translation,
            "cartesian_rotation": rotation,
            "gripper_position": self.current_gripper_state.position[0] if len(self.current_gripper_state.position) > 0 else 0.0
        }
        return True, robot_state_dict
        
    def _compute_home_override(self, b_pressed):
        """Compute smooth homing trajectory if B is held"""
        target_pos = np.array([0.4, -0.3, 0.15])
        target_quat = np.array([1.0, 0.3, 0.0, 0.0])
        target_quat = target_quat / np.linalg.norm(target_quat)

        # Use the last commanded pose as the starting point, or current physical state
        current_pos = None
        current_quat = None
        if self.controller.last_target is not None:
            current_pos = self.controller.last_target["pos"]
            current_quat = self.controller.last_target["quat"]
        elif self.controller.robot_state is not None:
            current_pos = self.controller.robot_state["pos"]
            current_quat = self.controller.robot_state["quat"]

        if current_pos is not None and current_quat is not None:
            # Distance calculation
            pos_diff = target_pos - current_pos
            pos_dist = np.linalg.norm(pos_diff)
            
            # Max speeds (per 1/15th of a second)
            MAX_LIN_SPEED = 0.10 # m/s
            MAX_ANG_SPEED = 0.2  # rad/s
            dt = 1.0 / 50.0
            max_pos_step = MAX_LIN_SPEED * dt
            max_quat_step = MAX_ANG_SPEED * dt
            
            # Linear step
            if pos_dist > max_pos_step:
                next_pos = current_pos + (pos_diff / pos_dist) * max_pos_step
            else:
                next_pos = target_pos

            # Slerp for quaternion
            dot = np.dot(current_quat, target_quat)
            if dot < 0.0:
                target_quat = -target_quat
                dot = -dot
            
            dot = np.clip(dot, -1.0, 1.0)
            theta_0 = np.arccos(dot)
            
            if theta_0 > max_quat_step:
                q3 = target_quat - current_quat * dot
                q3 = q3 / np.linalg.norm(q3)
                next_quat = current_quat * np.cos(max_quat_step) + q3 * np.sin(max_quat_step)
                next_quat = next_quat / np.linalg.norm(next_quat)
            else:
                next_quat = target_quat

            translation = next_pos
            rotation = next_quat
        else:
            translation = target_pos
            rotation = target_quat

        # Sync VR Policy internals so releasing B doesn't cause a Cartesian jump
        if self.controller.vr_state is not None:
            self.controller.robot_origin = {"pos": translation.copy(), "quat": rotation.copy()}
            self.controller.vr_origin = {
                "pos": self.controller.vr_state["pos"].copy(), 
                "quat": self.controller.vr_state["quat"].copy()
            }
        
        return translation, rotation

    def _publish_commands(self, translation, rotation, target_gripper):
        """Publish Cartesian Pose and Gripper Trajectory"""
        # Publish Pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position =  self.to_ros_point(translation)
        msg.pose.orientation = self.to_ros_quat(rotation)
        self.target_pose_publisher_.publish(msg)

        # Broadcast Target Pose TF for RViz
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.base_frame
        prefix = self.ns[1:] + "_" if len(self.ns) > 1 else ""
        t.child_frame_id = f"{prefix}vr_target_pose"
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(rotation[0])
        t.transform.rotation.y = float(rotation[1])
        t.transform.rotation.z = float(rotation[2])
        t.transform.rotation.w = float(rotation[3])
        self.tf_broadcaster.sendTransform(t)

        # Publish Gripper
        gripper_msg = JointTrajectory()
        gripper_msg.header.stamp = self.get_clock().now().to_msg()
        
        prefix = self.ns[1:] + "_" if self.ns != "" else ""
        gripper_msg.joint_names = [f"{prefix}rh_r1"]
        
        point = JointTrajectoryPoint()
        point.positions = [target_gripper * 1.1]
        
        # ~66ms duration for trajectory execution to match 15Hz loop
        point.time_from_start.nanosec = 20_000_000 # 66_666_666
        gripper_msg.points = [point]
        self.gripper_publisher_.publish(gripper_msg)
        return msg

    def _publish_debug_info(self, msg_header, controller_info, controller_action_info, final_translation, final_rotation):
        """Publish massive debugging message for data collection"""
        debug_msg = FMQDebug()
        debug_msg.header = msg_header
        debug_msg.movement_enabled = controller_info["movement_enabled"]

        debug_msg.vr_raw_pose.position =  self.to_ros_point(controller_action_info["vr_raw_pos"])
        debug_msg.vr_raw_pose.orientation = self.to_ros_quat(controller_action_info["vr_raw_quat"])

        debug_msg.vr_origin.position =  self.to_ros_point(controller_action_info["vr_origin_pos"])
        debug_msg.vr_origin.orientation = self.to_ros_quat(controller_action_info["vr_origin_quat"])

        debug_msg.vr_pose.position =  self.to_ros_point(controller_action_info["vr_pos"])
        debug_msg.vr_pose.orientation = self.to_ros_quat(controller_action_info["vr_quat"])

        debug_msg.robot_raw_pose.position =  self.to_ros_point(controller_action_info["robot_raw_pos"])
        debug_msg.robot_raw_pose.orientation = self.to_ros_quat(controller_action_info["robot_raw_quat"])

        debug_msg.robot_origin.position =  self.to_ros_point(controller_action_info["robot_origin_pos"])
        debug_msg.robot_origin.orientation = self.to_ros_quat(controller_action_info["robot_origin_quat"])

        debug_msg.robot_pose.position =  self.to_ros_point(controller_action_info["robot_pos"])
        debug_msg.robot_pose.orientation = self.to_ros_quat(controller_action_info["robot_quat"])

        debug_msg.robot_target_pose.position =  self.to_ros_point(controller_action_info["robot_target_pos"])
        debug_msg.robot_target_pose.orientation = self.to_ros_quat(controller_action_info["robot_target_quat"])
        
        if self.controller.last_target is not None:
            # 1. Position Delta
            delta_pos = final_translation - self.controller.last_target["pos"]
            
            # 2. Quaternion Delta (Relative Rotation)
            r_prev = Rotation.from_quat(self.controller.last_target["quat"])
            r_curr = Rotation.from_quat(final_rotation)
            
            # r_curr * r_prev.inv() gives the rotation needed to get from prev to curr
            r_diff = r_curr * r_prev.inv() 
            delta_quat = r_diff.as_quat()
            if delta_quat[3] < 0.0:
                delta_quat = -delta_quat
        else:
            delta_pos = np.zeros(3)
            delta_quat = np.array([0.0, 0.0, 0.0, 1.0])

        debug_msg.delta_target_pose.position = self.to_ros_point(delta_pos)
        debug_msg.delta_target_pose.orientation = self.to_ros_quat(delta_quat)
        
        self.debug_publisher_.publish(debug_msg)

    def timer_callback(self):
        controller_info = self.controller.get_info()
        a_pressed, b_pressed = self._handle_recording_triggers(controller_info)
        
        succ, robot_state_dict = self._get_current_robot_state()
        if not succ: return

        # 1. ALWAYS override with C++ feedback before calculating VR math
        if self.latest_cmd_pos is not None and controller_info["movement_enabled"]:
            robot_state_dict["cartesian_position"] = self.latest_cmd_pos.copy()
            robot_state_dict["cartesian_rotation"] = self.latest_cmd_quat.copy()

        # 2. Synchronous Poll (Calculates everything once)
        result = self.controller.poll_and_process(robot_state_dict)
        if result is None or result[2] == {}:
            return
        
        target_pose, target_gripper, controller_action_info = result

        # 3. Handle initialization/sync
        if self.controller.last_target is None:
            self.controller.last_target = {
                "pos": robot_state_dict["cartesian_position"].copy(), 
                "quat": robot_state_dict["cartesian_rotation"].copy()
            }

        just_stopped = controller_info.get("just_stopped", False)

        # 4. Gating (Idle State)
        if not controller_info["movement_enabled"] and not b_pressed:
            self.last_raw_target = None

            if just_stopped:  
                sync_pos = self.latest_cmd_pos.copy() if self.latest_cmd_pos is not None \
                   else robot_state_dict["cartesian_position"].copy()
                sync_quat = self.latest_cmd_quat.copy() if self.latest_cmd_quat is not None \
                            else robot_state_dict["cartesian_rotation"].copy()
                self.controller.last_target = {"pos": sync_pos, "quat": sync_quat}
                self.controller.robot_origin = {"pos": sync_pos, "quat": sync_quat}
                if hasattr(self.controller, 'vr_state') and self.controller.vr_state is not None:
                    self.controller.vr_origin = {
                        "pos": self.controller.vr_state["pos"].copy(),
                        "quat": self.controller.vr_state["quat"].copy()
                    }
            
            idle_pos = self.controller.last_target["pos"]
            idle_quat = self.controller.last_target["quat"]

            # Broadcast idle state to data collector natively at 15Hz
            debug_msg = FMQDebug()
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = self.base_frame
            debug_msg.movement_enabled = False
            self.debug_publisher_.publish(debug_msg)

            # Broadcast Target TF while idle (follows robot)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            prefix = self.ns[1:] + "_" if len(self.ns) > 1 else ""
            t.child_frame_id = f"{prefix}vr_target_pose"
            t.transform.translation.x = float(idle_pos[0])
            t.transform.translation.y = float(idle_pos[1])
            t.transform.translation.z = float(idle_pos[2])
            t.transform.rotation.x = float(idle_quat[0])
            t.transform.rotation.y = float(idle_quat[1])
            t.transform.rotation.z = float(idle_quat[2])
            t.transform.rotation.w = float(idle_quat[3])
            self.tf_broadcaster.sendTransform(t)
            
            return
        
        # Pull standard action intent from VR Controller
        # target_pose, target_gripper, controller_action_info = self.controller.forward(robot_state_dict)

        if not controller_action_info: # empty dict implies no poses from meta quest Yet
            self.get_logger().debug("VR Poses unavailable, skipping loop.")
            return

        if b_pressed:
            translation, rotation = self._compute_home_override(b_pressed)
        else:
            # Standard VR Teleop
            target_pos_vr = target_pose[:3]
            target_quat_vr = target_pose[3:]

            translation, rotation = self._apply_safety_shields(
                target_pos_vr, 
                target_quat_vr, 
                controller_action_info
            )
            
        # 5. Execute and Log
        pose_msg = self._publish_commands(translation, rotation, target_gripper)
        self._publish_debug_info(pose_msg.header, controller_info, controller_action_info, translation, rotation)
        self.controller.last_target = {"pos": translation.copy(), "quat": rotation.copy()}


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.controller.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()