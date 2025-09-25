#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from tf2_ros import Buffer, TransformListener
from franka_custom_msgs.msg import FMQDebug

try:
    from .oculus_controller import VRPolicy
    from .transformations import euler_to_quat
except:
    from oculus_controller import VRPolicy
    from transformations import euler_to_quat

class CartesianPosePublisher(Node):

    def __init__(self):
        super().__init__('cartesian_pose_publisher')

        self.ns = "/franka_right" #self.get_namespace()

        self.joint_subscriber_ = self.create_subscription(JointState, self.ns + '/joint_states', self.joint_state_callback, 1)
        self.publisher_ = self.create_publisher(PoseStamped, self.ns + '/target_cartesian_pose', 1)
        self.debug_publisher_ = self.create_publisher(FMQDebug, self.ns + '/mq_debug', 1)
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_joint_state = JointState()

        self.controller = VRPolicy()

        self.controller.reset_state()

        self.get_logger().info('VRPolicy Publisher started.')


    def joint_state_callback(self, msg):
        self.current_joint_state = msg


    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.ns[1:]+'_fr3_link0', self.ns[1:]+'_fr3_link8', rclpy.time.Time(), Duration(seconds=0.1))
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


    def timer_callback(self):

        controller_info = self.controller.get_info()
        skip_action = not controller_info["movement_enabled"]

        succ, translation, rotation = self.lookup_transform()
        if not succ:
            return
        
        state_dict = {}
        state_dict["cartesian_position"] = translation
        state_dict["cartesian_rotation"] = rotation
        state_dict["gripper_position"] = 0
        
        action, controller_action_info = self.controller.forward(state_dict)

        if skip_action:
            action = np.zeros(7)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.ns + '_fr3_link8'

        msg.pose.position = self.to_ros_point(action[0:3])
        msg.pose.orientation = self.to_ros_quat(euler_to_quat(action[3:-1]))
        
        self.publisher_.publish(msg)

        if controller_action_info != {}:

            debug_msg = FMQDebug()

            debug_msg.header = msg.header

            debug_msg.movement_enabled = controller_info["movement_enabled"]

            debug_msg.vr_raw_pose.position =  self.to_ros_point(controller_action_info["vr_raw_pos"])
            debug_msg.vr_raw_pose.orientation = self.to_ros_quat(controller_action_info["vr_raw_quat"])

            debug_msg.vr_origin.position =  self.to_ros_point(controller_action_info["vr_origin_pos"])
            debug_msg.vr_origin.orientation = self.to_ros_quat(controller_action_info["vr_origin_quat"])

            debug_msg.vr_target_pose.position =  self.to_ros_point(controller_action_info["vr_target_pos"])
            debug_msg.vr_target_pose.orientation = self.to_ros_quat(controller_action_info["vr_target_quat"])

            debug_msg.robot_raw_pose.position =  self.to_ros_point(controller_action_info["robot_raw_pos"])
            debug_msg.robot_raw_pose.orientation = self.to_ros_quat(controller_action_info["robot_raw_quat"])

            debug_msg.robot_origin.position =  self.to_ros_point(controller_action_info["robot_origin_pos"])
            debug_msg.robot_origin.orientation = self.to_ros_quat(controller_action_info["robot_origin_quat"])

            debug_msg.robot_target_pose.position =  self.to_ros_point(controller_action_info["robot_target_pos"])
            debug_msg.robot_target_pose.orientation = self.to_ros_quat(controller_action_info["robot_target_quat"])

            debug_msg.robot_action.position =  self.to_ros_point(controller_action_info["robot_action_pos"])
            debug_msg.robot_action.orientation = self.to_ros_quat(controller_action_info["robot_action_quat"])

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()