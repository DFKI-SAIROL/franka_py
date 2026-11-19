from typing import Any
import importlib
from abc import ABC, abstractmethod
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration


ACTION_PROCESSORS = {}

def register_action_processor(name: str):
    def decorator(cls):
        ACTION_PROCESSORS[name] = cls
        return cls
    return decorator


class BaseActionProcessor(ABC):
    def __init__(self, **config):
        self.config = config
        self.start_idx = config['start_idx']
        self.end_idx = config['end_idx']

    @abstractmethod
    def process(self, action_slice: np.ndarray):
        """Take a slice of the numpy array, convert it, and publish."""
        pass


@register_action_processor("DeltaJoint")
class DeltaJointProcessor(BaseActionProcessor):
    def __init__(self, **config):
        super().__init__(**config)
        # You can optionally define joint_names in your YAML if your C++ node requires them
        self.joint_names = config.get("joint_names", [])

    def process(self, action_slice: np.ndarray, stamp: Any) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.joint_names
        
        # Packing the raw delta directly into the position field as requested by your YAML
        msg.position = action_slice[self.start_idx:self.end_idx].tolist()
        
        return msg

@register_action_processor("PoseDelta6D")
class PoseDeltaProcessor(BaseActionProcessor):
    def __init__(self, **config):
        super().__init__(**config)
        self.base_frame = config.get("base_frame", "world")

    def process(self, action_array: np.ndarray, stamp: Any) -> PoseStamped:
        # 1. Apply the slice using the configured indices
        action_slice = action_array[self.start_idx : self.end_idx]
        
        # Ensure we actually have 9 elements for 6D Pose Delta (3 Pos, 6 Rot)
        if len(action_slice) < 9:
            return None
            
        tx, ty, tz = action_slice[0:3]
        r6d = action_slice[3:9]
        
        # Convert 6D to Quaternion
        x = r6d[0:3] / np.linalg.norm(r6d[0:3])
        y = r6d[3:6] - np.dot(r6d[3:6], x) * x
        y = y / np.linalg.norm(y)
        z = np.cross(x, y)
        delta_quat = Rotation.from_matrix(np.column_stack((x, y, z))).as_quat()
        
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        
        # 2. Publish raw delta for the C++ node to consume
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(tx), float(ty), float(tz)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = float(delta_quat[0]), float(delta_quat[1]), float(delta_quat[2]), float(delta_quat[3])
        
        return msg

    def execute(self, action_slice: np.ndarray) -> PoseStamped:
        tx, ty, tz = action_slice[0:3]
        r6d = action_slice[3:9]
        
        # Convert 6D to Quaternion
        x = r6d[0:3] / np.linalg.norm(r6d[0:3])
        y = r6d[3:6] - np.dot(r6d[3:6], x) * x
        y = y / np.linalg.norm(y)
        z = np.cross(x, y)
        delta_quat = Rotation.from_matrix(np.column_stack((x, y, z)))
        delta_pos = np.array([tx, ty, tz])
        
        if self.last_target is None:
            succ, curr_pos, curr_quat = self._lookup_transform()
            if not succ: return
            self.last_target = {"pos": curr_pos, "quat": curr_quat}
            
        target_pos = self.last_target["pos"] + delta_pos
        target_quat = (delta_quat * Rotation.from_quat(self.last_target["quat"])).as_quat()
        
        target_pos, target_quat = self._apply_safety_shields(target_pos, target_quat)
        self.last_target = {"pos": target_pos.copy(), "quat": target_quat.copy()}
        
        # Publish
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target_pos
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = target_quat
        return msg

@register_action_processor("PoseAbsolute")
class PoseAbsoluteProcessor(BaseActionProcessor):
    """Absolute end-effector target pose, e.g. for the safety layer's ``target_pose`` service.

    The action slice is position (3) followed by the orientation in ``rot_format``:
    ``rotvec`` (axis-angle, 3), ``quaternion`` (x, y, z, w, 4) or ``ee6d`` (first two rotation columns, 6).
    Optional ``workspace_min`` / ``workspace_max`` (3 values each, in ``frame_id``) clip the position.
    The last sent pose is kept in ``last_pose`` (position, quaternion xyzw), e.g. for homing.
    """

    ROT_DIMS = {"rotvec": 3, "quaternion": 4, "ee6d": 6}

    def __init__(self, **config):
        super().__init__(**config)
        self.frame_id = config.get("frame_id", "world")
        self.rot_format = config.get("rot_format", "rotvec")
        if self.rot_format not in self.ROT_DIMS:
            raise ValueError(f"rot_format must be one of {list(self.ROT_DIMS)}, got {self.rot_format!r}")
        if self.end_idx - self.start_idx != 3 + self.ROT_DIMS[self.rot_format]:
            raise ValueError(f"PoseAbsolute with {self.rot_format} needs {3 + self.ROT_DIMS[self.rot_format]} values")
        lo, hi = config.get("workspace_min"), config.get("workspace_max")
        self.workspace = (np.asarray(lo, float), np.asarray(hi, float)) if lo is not None and hi is not None else None
        self.last_pose = None

    def _rotation(self, values: np.ndarray) -> Rotation:
        if self.rot_format == "rotvec":
            return Rotation.from_rotvec(values)
        if self.rot_format == "quaternion":
            return Rotation.from_quat(values)
        x = values[0:3] / np.linalg.norm(values[0:3])
        y = values[3:6] - np.dot(values[3:6], x) * x
        y = y / np.linalg.norm(y)
        return Rotation.from_matrix(np.column_stack((x, y, np.cross(x, y))))

    def process(self, action_array: np.ndarray, stamp: Any) -> PoseStamped:
        values = np.asarray(action_array[self.start_idx:self.end_idx], dtype=float)
        if not np.all(np.isfinite(values)):
            return None
        position = values[:3]
        if self.workspace is not None:
            position = np.clip(position, *self.workspace)
        quat = self._rotation(values[3:]).as_quat()
        return self.make_msg(position, quat, stamp)

    def make_msg(self, position, quat_xyzw, stamp: Any) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in quat_xyzw)
        self.last_pose = (np.array(position, float), np.array(quat_xyzw, float))
        return msg


@register_action_processor("GripperAbsolute")
class GripperProcessor(BaseActionProcessor):
    def __init__(self, **config):
        super().__init__(**config)
        self.scale_factor = config.get('scale_factor', 1.0)
        
        # Attempt to pull joint_names from config, otherwise default to Franka standard
        self.joint_names = config.get('joint_names', ["franka_right_rh_r1"])
        
        # Since we decoupled from the node, pass frequency in via config or default to 30
        self.frequency = config.get('frequency', 30.0)

    def process(self, action_slice: np.ndarray, stamp: Any) -> JointTrajectory:
        msg = JointTrajectory()
        msg.header.stamp = stamp
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        # Scale the 1D network output
        point.positions = [float(action_slice[self.start_idx]) * self.scale_factor]
        
        # Set execution duration
        point.time_from_start.nanosec = int((1.0 / self.frequency) * 1e9)
        msg.points = [point]

        return msg