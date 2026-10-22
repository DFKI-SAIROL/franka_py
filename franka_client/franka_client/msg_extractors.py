from abc import ABC, abstractmethod
from typing import List, Sequence, Tuple

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R


MSG_EXTRACTOR = {}


def register_msg_extractor(name: str):
    """Decorator to easily add new extractors to the registry."""

    def decorator(cls):
        if name in MSG_EXTRACTOR:
            raise ValueError(f"A msg extractor named '{name}' is already registered!")
        MSG_EXTRACTOR[name] = cls
        return cls

    return decorator


class BaseMSGExtractor(ABC):
    def __init__(self, **config) -> None:
        self.config = config

    @property
    @abstractmethod
    def msgtype(self) -> str:
        """Message type of the processed message"""

    @property
    @abstractmethod
    def shape(self) -> Tuple:
        """Message type of the processed message"""

    @property
    @abstractmethod
    def dtype(self) -> str:
        """Message type of the processed message"""

    @property
    def name(self) -> str | List | None:
        """Message type of the processed message"""

    @abstractmethod
    def extract(self, msg):
        """Takes a deserialized ROS message and returns a dict of numpy arrays."""
        pass


@register_msg_extractor("sensor_msgs/msg/JointState")
class JointStateExtractor(BaseMSGExtractor):
    def __init__(self, **config):
        super().__init__(**config)
        self.fields = self.config.get("fields", ["position"])
        self.n_dof = self.config.get("n_dof", 1)

    @property
    def msgtype(self) -> str:
        return "sensor_msgs/msg/JointState"

    @property
    def shape(self) -> Tuple:
        return (len(self.fields) * self.n_dof,)

    @property
    def dtype(self) -> str:
        return "float32"

    def extract(self, msg):
        out = []

        # Flexibly pull only what the YAML requested
        if "position" in self.fields:
            out.append(np.array(msg.position, dtype=np.float32))
        if "velocity" in self.fields:
            out.append(np.array(msg.velocity, dtype=np.float32))
        if "effort" in self.fields:
            out.append(np.array(msg.effort, dtype=np.float32))
        return np.concatenate(out)


@register_msg_extractor("trajectory_msgs/msg/JointTrajectory")
class JointTrajectoryExtractor(BaseMSGExtractor):
    def __init__(self, **config):
        super().__init__(**config)
        # Fields can include position, velocity, accelerations, or effort
        self.fields = self.config.get("fields", ["positions"])
        self.n_dof = self.config.get("n_dof", 1)
        # Default to extracting the last point (the target) in the trajectory
        self.point_index = self.config.get("point_index", 0)

    @property
    def msgtype(self) -> str:
        return "trajectory_msgs/msg/JointTrajectory"

    @property
    def shape(self) -> Tuple:
        return (len(self.fields) * self.n_dof,)

    @property
    def dtype(self) -> str:
        return "float32"

    def extract(self, msg):
        if not msg.points:
            return np.zeros(self.shape, dtype=np.float32)

        # Select the specific trajectory point (usually the last one)
        point = msg.points[self.point_index]
        out = []

        # Note: JointTrajectoryPoint fields are plural (positions, velocities, etc.)
        if "positions" in self.fields or "position" in self.fields:
            out.append(np.array(point.positions, dtype=np.float32))

        if "velocities" in self.fields or "velocity" in self.fields:
            out.append(np.array(point.velocities, dtype=np.float32))

        if "accelerations" in self.fields:
            out.append(np.array(point.accelerations, dtype=np.float32))

        if "effort" in self.fields:
            out.append(np.array(point.effort, dtype=np.float32))

        # Handle cases where requested n_dof might not match msg length
        combined = (
            np.concatenate(out) if out else np.zeros(self.shape, dtype=np.float32)
        )

        return combined


@register_msg_extractor("geometry_msgs/msg/PoseStamped")
class PoseExtractor(BaseMSGExtractor):
    def __init__(self, **config):
        super().__init__(**config)

        # Flexibly allow extracting just position, just orientation, or both
        self.fields = self.config.get("fields", ["position", "orientation"])

        # Expected options: "quaternion", "euler", "ee6d"
        self.rot_format = self.config.get("rot_format", "quaternion")

    @property
    def msgtype(self) -> str:
        return "geometry_msgs/msg/PoseStamped"

    @property
    def shape(self) -> tuple:
        dim = 0
        if "position" in self.fields:
            dim += 3

        if "orientation" in self.fields:
            if self.rot_format == "quaternion":
                dim += 4
            elif self.rot_format == "euler":
                dim += 3
            elif self.rot_format == "ee6d":
                dim += 6
            else:
                raise ValueError(f"Unknown rot_format: {self.rot_format}")

        return (dim,)

    @property
    def dtype(self) -> str:
        return "float32"

    @property
    def name(self):
        return None

    def extract(self, msg):
        out = []

        # 1. Extract Position
        if "position" in self.fields:
            pos = msg.position
            out.append([pos.x, pos.y, pos.z])

        # 2. Extract and Convert Orientation
        if "orientation" in self.fields:
            q = msg.orientation
            # scipy.spatial.transform expects quaternions in [x, y, z, w] format
            quat = [q.x, q.y, q.z, q.w]

            if self.rot_format == "quaternion":
                out.append(quat)
            else:
                # Initialize scipy Rotation object
                rot = R.from_quat(quat)

                if self.rot_format == "euler":
                    # 'xyz' denotes extrinsic roll, pitch, yaw.
                    # Change to 'XYZ' if your lab expects intrinsic!
                    euler = rot.as_euler("xyz", degrees=False)
                    out.append(euler)

                elif self.rot_format == "ee6d":
                    # Standard 6D continuous representation (Zhou et al. 2019)
                    # We take the first two column vectors of the 3x3 rotation matrix.
                    rot_matrix = rot.as_matrix()

                    # rot_matrix[:, :2] gets the 3x2 matrix.
                    # .flatten('F') flattens it column by column: [x1, y1, z1, x2, y2, z2]
                    ee6d = rot_matrix[:, :2].flatten(order="F")
                    out.append(ee6d)

        # Concatenate into a flat 1D array
        return np.concatenate(out).astype(np.float32)


@register_msg_extractor("sensor_msgs/Image")
class ImageExtractor(BaseMSGExtractor):
    def __init__(self, **config):
        super().__init__(**config)

        self.center_crop = self.config.get("center_crop", None)
        self.resize = self.config.get("resize", None)
        self.encoding = self.config.get("encoding", "rgb8")

        # Determine the number of output channels for the shape property
        if self.encoding in ["rgb8", "bgr8"]:
            self.channels = 3
        elif self.encoding in ["rgba8", "bgra8"]:
            self.channels = 4
        elif self.encoding in ["mono8", "16UC1"]:
            self.channels = 1
        else:
            raise ValueError(f"Unsupported target encoding in config: {self.encoding}")

    @property
    def msgtype(self) -> str:
        return "sensor_msgs/Image"

    @property
    def shape(self) -> tuple:
        if self.resize:
            return (self.resize[0], self.resize[1], self.channels)
        if self.center_crop:
            return (self.center_crop[0], self.center_crop[1], self.channels)
        raise ValueError("Must provide resize or center_crop to infer shape.")

    @property
    def dtype(self) -> str:
        return "video"

    @property
    def name(self) -> List[str]:
        return ["height", "width", "channels"]

    def extract(self, msg):
        # 1. Determine shape and type from the RAW ROS encoding
        raw_encoding = msg.encoding
        if raw_encoding in ["rgb8", "bgr8"]:
            dtype = np.uint8
            raw_channels = 3
        elif raw_encoding in ["rgba8", "bgra8"]:
            dtype = np.uint8
            raw_channels = 4
        elif raw_encoding in ["mono8"]:
            dtype = np.uint8
            raw_channels = 1
        elif raw_encoding in ["16UC1"]:  # Common for depth cameras
            dtype = np.uint16
            raw_channels = 1
        else:
            raise ValueError(f"Unsupported incoming image encoding: {raw_encoding}")

        # 2. Convert raw bytes to a 3D NumPy array
        raw_data = msg.data
        if not isinstance(raw_data, np.ndarray):
            raw_data = np.frombuffer(raw_data, dtype=dtype)
        else:
            raw_data = raw_data.view(dtype)

        img = raw_data.reshape((msg.height, msg.width, raw_channels))

        # 3. Smart Target Conversions (Raw ROS Encoding -> Target YAML Encoding)
        if self.encoding == "rgb8":
            if raw_encoding == "bgr8":
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            elif raw_encoding == "rgba8":
                # Drops the alpha channel (4 channels -> 3 channels)
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
            elif raw_encoding == "bgra8":
                # Fixes color order AND drops the alpha channel
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            elif raw_encoding == "mono8":
                # Duplicates the grayscale channel across RGB
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        elif self.encoding == "rgba8":
            if raw_encoding == "bgra8":
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA)
            elif raw_encoding == "rgb8":
                # Adds a blank alpha channel if requested
                img = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA)

        if self.center_crop:
            h, w = img.shape[:2]

            if isinstance(self.center_crop, bool) and self.center_crop is True:
                # Dynamically find the shortest edge to create a perfect square
                min_dim = min(h, w)
                crop_h, crop_w = min_dim, min_dim
            else:
                crop_h, crop_w = self.center_crop

            start_y = max(0, h // 2 - crop_h // 2)
            start_x = max(0, w // 2 - crop_w // 2)
            img = img[start_y : start_y + crop_h, start_x : start_x + crop_w]

        # 5. Apply Resize
        if self.resize:
            target_h, target_w = self.resize
            img = cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_AREA)

            if self.channels == 1 and len(img.shape) == 2:
                img = np.expand_dims(img, axis=-1)
        img = img.transpose(2, 0, 1)

        return img


@register_msg_extractor("franka_custom_msgs/msg/FMQDebug")
class FMQDebugExtractor(BaseMSGExtractor):
    def __init__(self, **config):
        super().__init__(**config)

        # Normalize fields to a list
        fields = self.config.get("fields", ["delta_target_pose"])
        self.fields = [fields] if isinstance(fields, str) else fields

        # Determine mode
        self.is_trigger = "movement_enabled" in self.fields

        if not self.is_trigger:
            # Instantiate a single PoseExtractor to process all requested poses
            pose_config = {
                # Map the user's "pose_fields" in YAML to the child's "fields"
                "fields": self.config.get("pose_fields", ["position", "orientation"]),
                "rot_format": self.config.get("rot_format", "quaternion"),
            }
            self.pose_extractor = MSG_EXTRACTOR["geometry_msgs/msg/PoseStamped"](
                **pose_config
            )

    @property
    def msgtype(self) -> str:
        return "franka_custom_msgs/msg/FMQDebug"

    @property
    def shape(self) -> tuple:
        if self.is_trigger:
            return (1,)

        # If we asked for ["vr_pose", "delta_target_pose"], the dimension is simply
        # 2 * (the shape of one configured pose)
        total_dim = len(self.fields) * self.pose_extractor.shape[0]
        return (total_dim,)

    @property
    def dtype(self) -> str:
        # bool_ is preferred for standard HuggingFace/numpy boolean arrays
        return "bool_" if self.is_trigger else "float32"

    def extract(self, msg):
        if self.is_trigger:
            # Return raw boolean if used as a trigger
            return msg.movement_enabled

        out = []
        for field_name in self.fields:
            # Magic python function: dynamically gets msg.vr_pose, msg.delta_target_pose, etc.
            pose_msg = getattr(msg, field_name)

            # Delegate the heavy lifting to the PoseExtractor
            extracted_array = self.pose_extractor.extract(pose_msg)
            out.append(extracted_array)

        return np.concatenate(out)


@register_msg_extractor("franka_custom_msgs/msg/RigSnapshot")
class RigSnapshotMSGExtractor(BaseMSGExtractor):
    def __init__(self, **config) -> None:
        super().__init__(**config)
        self.image_extractor = ImageExtractor(**config)

        self.camera_name = self.config.get("camera_name", None)
        if self.camera_name is None:
            raise ValueError("Please define a camera from which to extract the image.")
        elif not isinstance(self.camera_name, str):
            raise ValueError(
                f"The extractor class extracts images for a single camera, but multiple ({self.camera_name}) were given."
            )

    @property
    def msgtype(self) -> str:
        return "franka_custom_msgs/msg/RigSnapshot"

    @property
    def shape(self) -> Tuple:
        return self.image_extractor.shape

    @property
    def dtype(self) -> str:
        return self.image_extractor.dtype

    @property
    def name(self) -> str | List | None:
        return self.image_extractor.name

    def extract(self, msg):
        for cam_name, img_msg in zip(msg.camera_names, msg.rgbs):
            if cam_name == self.camera_name:
                return self.image_extractor.extract(img_msg)


@register_msg_extractor("franka_custom_msgs/msg/FIJKDebug")
class FIJKDebugExtractor(BaseMSGExtractor):
    def __init__(self, **config):
        super().__init__(**config)

        fields = self.config.get("fields", ["q_actual", "dq_commanded", "q_target"])
        self.fields = [fields] if isinstance(fields, str) else fields

        self.n_dof = self.config.get("n_dof", 7)

    @property
    def msgtype(self) -> str:
        return "franka_custom_msgs/msg/FIJKDebug"

    @property
    def shape(self) -> tuple:
        return (len(self.fields) * self.n_dof,)

    @property
    def dtype(self) -> str:
        return "float32"

    def extract(self, msg):
        out = []
        for field_name in self.fields:
            data = getattr(msg, field_name)
            out.append(np.array(data, dtype=np.float32))

        return np.concatenate(out)


_PC2_DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32,
               7: np.float32, 8: np.float64}


@register_msg_extractor("sensor_msgs/msg/PointCloud2")
class PointCloudExtractor(BaseMSGExtractor):
    """sensor_msgs/PointCloud2 -> (num_points, 3 | 6) float32: xyz [+ rgb in 0..1].

    Config:
        num_points: 4096            fixed size of the output
        fields: xyz | xyzrgb
        sampling: random | voxel | fps
                    random: uniform over the points (dense regions keep more points)
                    voxel:  one point per cell of a grid sized for ~num_points cells, i.e. even
                            coverage like FPS at a fraction of the cost (~10-40 ms for 80k points)
                    fps:    farthest point sampling on a random preselection of fps_candidates
                            (default 2 * num_points) points; ~0.1-0.3 s for 4096, offline use
        crop: {min: [x, y, z], max: [x, y, z]}   box in the cloud's frame, applied first
        seed: 0

    Clouds with fewer points are padded by repeating points; an empty cloud gives zeros.
    """

    def __init__(self, **config):
        super().__init__(**config)
        self.num_points = int(self.config.get("num_points", 4096))
        self.fields = self.config.get("fields", "xyz")
        if self.fields not in ("xyz", "xyzrgb"):
            raise ValueError(f"fields must be xyz or xyzrgb, not {self.fields!r}")
        self.sampling = self.config.get("sampling", "random")
        if self.sampling not in ("random", "voxel", "fps"):
            raise ValueError(f"sampling must be random, voxel or fps, not {self.sampling!r}")
        self.fps_candidates = int(self.config.get("fps_candidates", 2 * self.num_points))
        self._voxel = None  # grid size of the last cloud, the start of the next search
        crop = self.config.get("crop")
        self.crop = (np.asarray(crop["min"], np.float32), np.asarray(crop["max"], np.float32)) if crop else None
        self.rng = np.random.default_rng(self.config.get("seed", 0))

    @property
    def msgtype(self) -> str:
        return "sensor_msgs/msg/PointCloud2"

    @property
    def shape(self) -> tuple:
        return (self.num_points, 6 if self.fields == "xyzrgb" else 3)

    @property
    def dtype(self) -> str:
        return "float32"

    @property
    def name(self) -> List[str]:
        return ["x", "y", "z", "r", "g", "b"] if self.fields == "xyzrgb" else ["x", "y", "z"]

    @staticmethod
    def decode(msg) -> Tuple[np.ndarray, np.ndarray | None]:
        """(N, 3) float32 xyz with NaNs dropped and (N, 3) uint8 rgb (None if the cloud has none)."""
        names = [f.name for f in msg.fields]
        dtype = np.dtype({
            "names": names,
            "formats": [np.dtype(_PC2_DTYPES[f.datatype]).newbyteorder(">" if msg.is_bigendian else "<")
                        for f in msg.fields],
            "offsets": [f.offset for f in msg.fields],
            "itemsize": msg.point_step,
        })
        data = np.frombuffer(memoryview(msg.data), dtype=dtype, count=msg.width * msg.height)
        xyz = np.stack([data["x"], data["y"], data["z"]], axis=-1).astype(np.float32)
        ok = np.isfinite(xyz).all(axis=1)
        rgb = None
        key = "rgb" if "rgb" in names else "rgba" if "rgba" in names else None
        if key:
            packed = np.ascontiguousarray(data[key]).view(np.uint32)
            rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], axis=-1).astype(np.uint8)[ok]
        return xyz[ok], rgb

    def _fps(self, xyz: np.ndarray, k: int) -> np.ndarray:
        n = len(xyz)
        if n > self.fps_candidates:
            pre = self.rng.choice(n, self.fps_candidates, replace=False)
        else:
            pre = np.arange(n)
        x, y, z = (np.ascontiguousarray(xyz[pre, i]) for i in range(3))
        chosen = np.empty(k, np.int64)
        dist = np.full(len(pre), np.inf, np.float32)
        d, t = np.empty_like(dist), np.empty_like(dist)
        chosen[0] = self.rng.integers(len(pre))
        for i in range(1, k):  # in-place: no temporaries per iteration
            j = chosen[i - 1]
            np.subtract(x, x[j], out=d)
            np.multiply(d, d, out=d)
            np.subtract(y, y[j], out=t)
            np.multiply(t, t, out=t)
            d += t
            np.subtract(z, z[j], out=t)
            np.multiply(t, t, out=t)
            d += t
            np.minimum(dist, d, out=dist)
            chosen[i] = int(np.argmax(dist))
        return pre[chosen]

    def _voxel_sample(self, xyz: np.ndarray, k: int) -> np.ndarray:
        """One random point per occupied cell, with the cell size searched so that >= k cells."""
        lo = xyz.min(axis=0)
        span = float(np.max(xyz.max(axis=0) - lo)) or 1.0
        voxel = self._voxel or span / np.cbrt(k)
        order = self.rng.permutation(len(xyz))  # random representative per cell
        pts = xyz[order] - lo
        best = None
        for _ in range(8):
            keys = np.floor(pts / voxel).astype(np.int64)
            dims = keys.max(axis=0) + 1
            flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
            _, first = np.unique(flat, return_index=True)
            if len(first) >= k:
                best = first
                if len(first) < 1.5 * k:
                    break
                voxel *= 1.25
            else:
                voxel *= 0.7
        self._voxel = voxel
        if best is None:  # still fewer cells than k: every point is its own cell
            return order
        return order[self.rng.choice(best, k, replace=False)]

    def extract(self, msg):
        xyz, rgb = self.decode(msg)
        if self.crop is not None and len(xyz):
            keep = np.all((xyz >= self.crop[0]) & (xyz <= self.crop[1]), axis=1)
            xyz, rgb = xyz[keep], (rgb[keep] if rgb is not None else None)
        out = np.zeros(self.shape, np.float32)
        n = len(xyz)
        if n == 0:
            return out
        if n >= self.num_points:
            if self.sampling == "fps":
                idx = self._fps(xyz, self.num_points)
            elif self.sampling == "voxel":
                idx = self._voxel_sample(xyz, self.num_points)
            else:
                idx = self.rng.choice(n, self.num_points, replace=False)
        else:  # pad by repeating points
            idx = np.concatenate([np.arange(n), self.rng.choice(n, self.num_points - n, replace=True)])
        out[:, :3] = xyz[idx]
        if self.fields == "xyzrgb" and rgb is not None:
            out[:, 3:] = rgb[idx].astype(np.float32) / 255.0
        return out
