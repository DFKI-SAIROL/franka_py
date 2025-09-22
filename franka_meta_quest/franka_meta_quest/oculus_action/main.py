#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

try:
    from .oculus_controller import VRPolicy
    from .transformations import euler_to_quat
except:
    from oculus_controller import VRPolicy
    from transformations import euler_to_quat

class CartesianPosePublisher(Node):

    def __init__(self):
        super().__init__('cartesian_pose_publisher')

        self.joint_subscriber_ = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 1)
        self.publisher_ = self.create_publisher(PoseStamped, 'target_cartesian_pose', 1)
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ns = "/franka_right" #self.get_namespace()
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

        msg.pose.position.x = action[0]
        msg.pose.position.y = action[1]
        msg.pose.position.z = action[2]

        quat = euler_to_quat(action[3:-1])
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.publisher_.publish(msg)


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