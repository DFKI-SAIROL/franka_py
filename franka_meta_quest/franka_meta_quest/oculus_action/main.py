#!/usr/bin/env python3
import math
import numpy as np
import threading
import sys
import termios


import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Time, Duration as ROSDuration
from tf2_ros import Buffer, TransformListener
from franka_custom_msgs.msg import FMQDebug
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
        self.declare_parameter('end_effector_frame', 'fr3_link8')
        
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.end_effector_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        
        self.base_frame = "world"
        self.end_effector_frame = "rh_p12_rn_base"

        print(self.base_frame)
        print(self.end_effector_frame)

        # fallback for bimanual prefix parsing if left default
        if self.base_frame == 'base' and self.ns != '':
            self.base_frame = self.ns[1:] + '_' + self.base_frame
        if self.ns != '':
            self.end_effector_frame = self.ns[1:] + '_' + self.end_effector_frame

        self.joint_subscriber_ = self.create_subscription(JointState, self.ns + '/franka/joint_states', self.joint_state_callback, 1)
        self.gripper_subscriber_ = self.create_subscription(JointState, self.ns + '/franka_gripper/joint_states', self.gripper_state_callback, 1)
        self.publisher_ = self.create_publisher(PoseStamped, self.ns + '/target_pose', 1)
        self.gripper_publisher_ = self.create_publisher(JointTrajectory, self.ns + '/gripper/gripper_controller/joint_trajectory', 1)
        self.debug_publisher_ = self.create_publisher(FMQDebug, self.ns + '/mq_debug', 1)
        # TODO: add YAML
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.init_transform = None

        self.current_joint_state = JointState()
        self.current_gripper_state = JointState()

        self.controller = VRPolicy()

        self.controller.reset_state()
        
        # Service Clients for Data Collection
        self.cli_rec = self.create_client(Trigger, '/data_collector/record_data_trigger')
        
        # State tracking for buttons
        self.last_buttons = {"A": False, "B": False, "X": False, "Y": False}

        self.get_logger().info('VRPolicy Publisher started.')


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
            if hasattr(self, 'last_raw_target'):
                del self.last_raw_target # Reset raw tracker on button release
            return target_pos, target_quat

        if not hasattr(self, 'last_raw_target') or self.controller.last_target is None:
            self.last_raw_target = {"pos": vr_pos.copy(), "quat": target_quat.copy()}
            return target_pos, target_quat

        # 1. OUTLIER REJECTION: Check instant jumps on the unscaled VR tracking offsets
        raw_pos_diff = vr_pos - self.last_raw_target["pos"]
        raw_pos_dist = np.linalg.norm(raw_pos_diff)

        # Always update the raw tracker memory so the headset can recover from glitches
        self.last_raw_target = {"pos": vr_pos.copy(), "quat": target_quat.copy()}
        
        # Max speeds (per 1/15th of a second)
        MAX_POS_STEP = 0.03   # 45 cm/s robot limit
        MAX_QUAT_STEP = 0.15  # 135 deg/s robot limit
        
        # Outlier Rejection: 0.08m per VR tick = 1.2 m/s jumping. 
        if raw_pos_dist > 0.08:
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

    def timer_callback(self):

        controller_info = self.controller.get_info()
        
        # === Button Triggers for Recording ===
        # Mapping: A/X -> Start, B/Y -> Stop
        # Detect rising edge
        a_pressed = controller_info.get("success", False) # A or X
        b_pressed = controller_info.get("failure", False) # B or Y
        
        if a_pressed and not self.last_buttons["A"]: # Using "A" key for success logic state
             self.call_service_async(self.cli_rec, 'Recording')
             
        self.last_buttons["A"] = a_pressed
        self.last_buttons["B"] = b_pressed
        
        # === Gating ===
        # if not lower lever pressed ("movement_enabled"), do not update target. 
        # Update: We do NOT publish at all if movement is not enabled.
        if not controller_info["movement_enabled"] and not b_pressed:
             return

        succ, translation, rotation = self.lookup_transform()
        if not succ:
            return
        
        robot_state_dict = {}
        robot_state_dict["cartesian_position"] = translation
        robot_state_dict["cartesian_rotation"] = rotation
        
        # Read the actual physical gripper position if available, else 0
        current_gripper_pos = 0.0
        if len(self.current_gripper_state.position) > 0:
            current_gripper_pos = self.current_gripper_state.position[0]
        robot_state_dict["gripper_position"] = current_gripper_pos
        
        target_pose, target_gripper, controller_action_info = self.controller.forward(robot_state_dict)

        if controller_action_info == {}:
            print(self.ns, "empty poses", flush=True)
            return

        # === B BUTTON HOME POSITION OVERRIDE ===
        if b_pressed:
            target_pos = np.array([0.3, -0.3, 0.4])
            target_quat = np.array([0.95, 0.32, 0.0, 0.0])
            target_quat = target_quat / np.linalg.norm(target_quat)

            # Use the last commanded pose as the starting point, or the current robot_state if we don't have one
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
                MAX_LIN_SPEED = 0.10 # m/s (10 cm per second)
                MAX_ANG_SPEED = 0.2  # rad/s (~11 degrees per second)
                dt = 1.0 / 15.0
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
            self.controller.last_target = {"pos": translation.copy(), "quat": rotation.copy()}

        else:
            # === STANDARD VR TELEOPERATION ===
            target_pos_vr = target_pose[:3]
            target_quat_vr = target_pose[3:]

            target_pos_safe, target_quat_safe = self._apply_safety_shields(
                target_pos_vr, 
                target_quat_vr, 
                controller_action_info
            )
            
            # Make sure VRPolicy knows where the robot actually ended up command-wise
            self.controller.last_target = {"pos": target_pos_safe.copy(), "quat": target_quat_safe.copy()}
            
            translation = target_pos_safe
            rotation = target_quat_safe

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position =  self.to_ros_point(translation)
        msg.pose.orientation = self.to_ros_quat(rotation)
        self.publisher_.publish(msg)

        # Publish Gripper
        gripper_msg = JointTrajectory()
        gripper_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Determine correct prefrixed joint name
        prefix = self.ns[1:] + "_" if self.ns != "" else ""
        gripper_msg.joint_names = [f"{prefix}rh_r1"]
        
        point = JointTrajectoryPoint()
        # Scale 0-1 from VR trigger to 0-1.1 radians for Dynamixel (0=Open, 1.1=Closed)
        # Assumes target_gripper 1.0 = fully pressed = fully closed.
        scaled_gripper_target = target_gripper * 1.1 
        point.positions = [scaled_gripper_target]
        
        # ~66ms duration for trajectory execution to match 15Hz loop
        point.time_from_start.nanosec = 66666666
        gripper_msg.points = [point]
        self.gripper_publisher_.publish(gripper_msg)

        if controller_action_info != {}:

            debug_msg = FMQDebug()

            debug_msg.header = msg.header

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


            self.debug_publisher_.publish(debug_msg)


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