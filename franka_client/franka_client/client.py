import argparse
import importlib
import os
import yaml
import threading
from collections import deque
from queue import Queue
import time
import pickle
from dataclasses import dataclass
import math

import grpc
import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from pynput import keyboard
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation, Slerp
from tf2_ros import Buffer, TransformListener

import services_pb2
import services_pb2_grpc
from lerobot_utils import (
    grpc_channel_options,
    send_bytes_in_chunks,
    RemotePolicyConfig,
    FPSTracker,
    get_logger,
)

from msg_extractors import MSG_EXTRACTOR
from action_processors import ACTION_PROCESSORS

# --- Custom NumPy-based Data Classes ---
Action = np.ndarray
RawObservation = dict[str, np.ndarray]

@dataclass
class TimedData:
    timestamp: float
    timestep: int

    def get_timestamp(self):
        return self.timestamp

    def get_timestep(self):
        return self.timestep

@dataclass
class TimedAction(TimedData):
    action: Action

    def get_action(self):
        return self.action

@dataclass
class TimedObservation(TimedData):
    observation: RawObservation
    must_go: bool = False

    def get_observation(self):
        return self.observation
# ---------------------------------------

class RobiClient(Node):
    def __init__(self, config_file_path: str):
        with open(config_file_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Initialize node and logger
        node_name = self.config.get('node_name', 'robi_client')
        super().__init__(node_name)
        self.logger = get_logger(node_name)

        self.frequency = self.config.get('frequency', 30.0)
        self.task = self.config.get('task', "Pick and Place")

        # Storage for latest PROCESSED data: {logical_name: [arrays]}
        self.latest_data = {}
        self.latest_data_lock = threading.Lock()
        self.expected_data_keys = set()

        # Optional observation history: every `stride` control steps the latest data is stored, and each
        # observation carries the last `length` of them stacked along a new first axis (oldest first).
        history_cfg = self.config.get('observation_history', {}) or {}
        self.history_length = int(history_cfg.get('length', 1))
        self.history_stride = int(history_cfg.get('stride', 1))
        self.history = deque(maxlen=self.history_length)
        self._history_tick = 0

        # Setup subscribers
        self.callback_group = ReentrantCallbackGroup()
        self.subs = []
        self._setup_extractors_and_subs(self.config.get('mappings', {}))
        
        # Setup publishers
        self.pubs, self.action_processors = [], []
        self._setup_processors_and_pubs(self.config.get("actions", []))
        
        # State & keyboard control
        self.policy_active = False
        self.home_pose_cfg = self.config.get('home_pose', {})
        # Home target: a PoseStamped topic, or a SetPoseStamped service (e.g. the safety layer's target_pose).
        # With a service the motion is rate limited (max_linear_speed, max_angular_speed) and starts at the
        # last pose sent by a PoseAbsolute action processor; before the first one, at the current pose of
        # home_pose.ee_frame in frame_id from TF.
        self.home_pub, self.home_cli, self.home_path = None, None, None
        self.tf_buffer = None
        if self.home_pose_cfg.get('ee_frame'):
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.home_pose_cfg.get('service'):
            self.home_cli = (*self._create_service_client(
                self.home_pose_cfg['service'],
                self.home_pose_cfg.get('srvtype', 'franka_custom_msgs/srv/SetPoseStamped')),
                self.home_pose_cfg.get('request_field', 'pose'))
        else:
            self.home_pub = self.create_publisher(
                PoseStamped,
                self.home_pose_cfg.get('topic'),
                1
            )
        self.home_gripper_pub = self.create_publisher(
            JointTrajectory,
            self.home_pose_cfg.get('gripper_topic', '/franka_right/gripper/gripper_controller/joint_trajectory'),
            1
        )
        self.get_logger().info("Keyboard Controls: [R]un Policy | [S]top Policy | [H]ome Pose")
        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.start()

        # Client setup
        self.action_queue = Queue()
        self.action_queue_lock = threading.Lock()
        self.latest_action = -1
        self.latest_action_lock = threading.Lock()

        # Chunk timing — handled entirely client-side
        yaml_policy_config = self.config.get('policy', {})
        self.actions_per_chunk: int = yaml_policy_config.get('actions_per_chunk', 32)
        # How many actions are approximately sent to the robot during one inference pass
        policy_inference_fps: float = self.config.get('policy_inference_fps', 10.0)
        self.act_while_inference: int = max(1, math.ceil(self.frequency / policy_inference_fps))
        # Threshold: request a new observation when the queue drops to this many actions remaining
        default_threshold = self.actions_per_chunk - self.act_while_inference
        self._chunk_size_threshold: int = yaml_policy_config.get('chunk_size_threshold', default_threshold)
        self.logger.info(
            f"Chunk timing | actions_per_chunk={self.actions_per_chunk} | "
            f"act_while_inference={self.act_while_inference} | "
            f"chunk_size_threshold={self._chunk_size_threshold}"
        )

        # gRPC Setup
        self.server_address = self.config.get('server_address')
        self.channel = grpc.insecure_channel(self.server_address, grpc_channel_options())
        self.stub = services_pb2_grpc.AsyncInferenceStub(self.channel)
        
        # start client and handshake with server
        self.policy_config = RemotePolicyConfig(
            yaml_policy_config.get('policy_type'),
            yaml_policy_config.get('pretrained_name_or_path'),
            yaml_policy_config.get('lerobot_features'),
            self.actions_per_chunk,
            yaml_policy_config.get('policy_device'),
            fps=self.frequency,  # server uses this to derive environment_dt
        )
        self.start_client_server_connection()
        self.is_running = True
        
        # Start the Action Receiver Thread
        self.receive_thread = threading.Thread(target=self.receive_actions, daemon=True)
        self.receive_thread.start()

        # FPS measurement
        self.fps_tracker = FPSTracker(target_fps=self.frequency)
        self.must_go = threading.Event()
        self.must_go.set()

        # Set up the Control Loop
        timer_period = 1.0 / self.frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop_step)

    def start_client_server_connection(self):
        """Start the robot client and connect to the policy server"""
        try:
            # client-server handshake
            start_time = time.perf_counter()
            self.stub.Ready(services_pb2.Empty())
            end_time = time.perf_counter()
            self.logger.debug(f"Connected to policy server in {end_time - start_time:.4f}s")

            # send policy instructions
            policy_config_bytes = pickle.dumps(self.policy_config)
            policy_setup = services_pb2.PolicySetup(data=policy_config_bytes)

            self.logger.info("Sending policy instructions to policy server")
            self.logger.debug(
                f"Policy type: {self.policy_config.policy_type} | "
                f"Pretrained name or path: {self.policy_config.pretrained_name_or_path} | "
                f"Device: {self.policy_config.device}"
            )

            self.stub.SendPolicyInstructions(policy_setup)

            return True

        except grpc.RpcError as e:
            self.logger.error(f"Failed to connect to policy server: {e}")
            return False

    def receive_actions(self, verbose: bool = False):
        """Receive actions from the policy server"""
        self.logger.info("Action receiving thread starting")

        while self.is_running:
            try:
                actions_chunk = self.stub.GetActions(services_pb2.Empty())
                if len(actions_chunk.data) == 0:
                    continue 

                timed_actions = pickle.loads(actions_chunk.data)  # nosec
                self._aggregate_action_queues(timed_actions, self.config.get('aggregate_fn'))
                self.must_go.set() 

            except grpc.RpcError as e:
                self.logger.error(f"Error receiving actions: {e}")

    def _aggregate_action_queues(self, incoming_actions: list[TimedAction], aggregate_fn=None):
        """Finds the same timestep actions in the queue and aggregates them using the aggregate_fn"""
        if aggregate_fn is None:
            # default aggregate function: take the latest action
            def aggregate_fn(x1, x2): return x2

        future_action_queue = Queue()
        with self.action_queue_lock:
            internal_queue = list(self.action_queue.queue)

        current_action_queue = {action.get_timestep(): action.get_action() for action in internal_queue}

        for new_action in incoming_actions:
            with self.latest_action_lock:
                latest_action = self.latest_action

            # New action is older than the latest action in the queue, skip it
            if new_action.get_timestep() <= latest_action:
                continue

            # If the new action's timestep is not in the current action queue, add it directly
            elif new_action.get_timestep() not in current_action_queue:
                future_action_queue.put(new_action)
                continue

            # If the new action's timestep is in the current action queue, aggregate it
            future_action_queue.put(
                TimedAction(
                    timestamp=new_action.get_timestamp(),
                    timestep=new_action.get_timestep(),
                    action=aggregate_fn(
                        current_action_queue[new_action.get_timestep()], new_action.get_action()
                    ),
                )
            )

        with self.action_queue_lock:
            self.action_queue = future_action_queue

    def send_observation(self, obs: TimedObservation) -> bool:
        """Send observation to the policy server.
        Returns True if the observation was sent successfully, False otherwise."""
        # Assuming self.running is a flag that indicates if the client is active
        # if not self.running:
        #     raise RuntimeError("Client not running. Run RobotClient.start() before sending observations.")

        if not isinstance(obs, TimedObservation):
            raise ValueError("Input observation needs to be a TimedObservation!")

        start_time = time.perf_counter()
        observation_bytes = pickle.dumps(obs)
        serialize_time = time.perf_counter() - start_time
        self.logger.info(f"Observation serialization time: {serialize_time:.6f}s")

        try:
            observation_iterator = send_bytes_in_chunks(
                observation_bytes,
                services_pb2.Observation,
                log_prefix="[CLIENT] Observation",
                silent=True,
            )
            _ = self.stub.SendObservations(observation_iterator)
            obs_timestep = obs.get_timestep()
            self.logger.debug(f"Sent observation #{obs_timestep} | ")

            return True

        except grpc.RpcError as e:
            self.logger.error(f"Error sending observation #{obs.get_timestep()}: {e}")
            return False

    def _ready_to_send_observation(self):
        """Flags when the client is ready to send an observation"""
        if not self.policy_active:
            return False

        # 1. Check Action Queue Depth
        # Send a new observation when remaining actions fall at or below the threshold,
        # which is the number of actions still in-flight during one inference pass.
        with self.action_queue_lock:
            is_queue_low = self.action_queue.qsize() <= self._chunk_size_threshold

        if not is_queue_low:
            return False

        # 2. Check for ALL Essential Sensor Data (No Nones in lists)
        with self.latest_data_lock:
            for feature_key, array_list in self.latest_data.items():
                if any(v is None for v in array_list):
                    return False

        return self.history_length == 1 or len(self.history) > 0

    def _snapshot(self) -> RawObservation | None:
        """Latest data per feature key (topics of one key concatenated), or None while data is missing."""
        snapshot: RawObservation = {}
        with self.latest_data_lock:
            for feature_key in self.expected_data_keys:
                # self.latest_data[feature_key] is a list of arrays
                arrays = self.latest_data[feature_key]
                if any(v is None for v in arrays):
                    return None

                # If we are mapping multiple topics to one key (e.g., arm + gripper),
                # concatenate them. Otherwise, just grab the single array.
                if len(arrays) == 1:
                    snapshot[feature_key] = arrays[0]
                else:
                    snapshot[feature_key] = np.concatenate(arrays, axis=-1).astype(np.float32)
        return snapshot

    def _update_history(self):
        if self.history_length == 1:
            return
        if self._history_tick % self.history_stride == 0:
            snapshot = self._snapshot()
            if snapshot is not None:
                self.history.append(snapshot)
        self._history_tick += 1

    def build_observation(self, task: str | None = None) -> TimedObservation:
        """Assembles the observation automatically based on the YAML mappings"""
        if self.history_length == 1:
            raw_observation: RawObservation = self._snapshot()
        else:
            # (length, ...) per key, oldest first; padded with the oldest entry until the history is full
            frames = list(self.history)
            frames = [frames[0]] * (self.history_length - len(frames)) + frames
            raw_observation = {k: np.stack([f[k] for f in frames]) for k in frames[-1]}

        raw_observation["task"] = task if task is not None else self.task

        with self.latest_action_lock:
            latest_action = self.latest_action

        observation = TimedObservation(
            timestamp=time.time(),
            observation=raw_observation,
            timestep=max(latest_action, 0),
        )

        with self.action_queue_lock:
            observation.must_go = self.must_go.is_set() and self.action_queue.empty()

        return observation

    def execute_actions(self, action_array):
        """Dispatches the raw array to the dynamic processors and publishes them."""
        stamp = self.get_clock().now().to_msg()
        
        for pub, processor in zip(self.pubs, self.action_processors):
            # The processor takes the full array, slices it internally based on its config,
            # and returns the correct ROS message (or None if invalid)
            msg = processor.process(action_array, stamp)

            if msg is not None:
                self._send(pub, msg)

    def _send(self, output, msg):
        """Publish on a topic, or call a service (output = (client, request class, request field))."""
        if not isinstance(output, tuple):
            output.publish(msg)
            return
        cli, request_class, field = output
        if not cli.service_is_ready():
            self.get_logger().warn(f"Service {cli.srv_name} not available, action not sent",
                                   throttle_duration_sec=2.0)
            return
        request = request_class()
        setattr(request, field, msg)
        cli.call_async(request)

    def _create_service_client(self, name: str, srv_type_str: str):
        pkg, _, cls_name = srv_type_str.split('/')
        srv_class = getattr(importlib.import_module(f'{pkg}.srv'), cls_name)
        return self.create_client(srv_class, name, callback_group=self.callback_group), srv_class.Request

    def control_loop_step(self):
        self._update_history()
        if self.home_path:
            self._home_step()
        if not self.policy_active:
            # Drain the queue so old actions don't execute when we resume
            with self.action_queue_lock:
                while not self.action_queue.empty():
                    self.action_queue.get_nowait()
            return

        # 1. Execute Action
        action_pulled = None
        with self.action_queue_lock:
            if not self.action_queue.empty():
                timed_action = self.action_queue.get_nowait()
                action_array = timed_action.get_action()
                
                self.execute_actions(action_array)
                action_pulled = timed_action
                
        if action_pulled is not None:
            with self.latest_action_lock:
                self.latest_action = action_pulled.get_timestep()

        # 2. Check if we need to request new actions from server
        if self._ready_to_send_observation():
            # Build your observation from self.latest_data dict
            obs = self.build_observation()
            
            # Print FPS occasionally if verbose
            fps_metrics = self.fps_tracker.calculate_fps_metrics(obs.get_timestamp())
            if obs.get_timestep() % 30 == 0:
                 self.logger.info(
                    f"Obs #{obs.get_timestep()} | Avg FPS: {fps_metrics['avg_fps']:.2f}"
                )

            # Non-blocking gRPC send
            self.send_observation(obs)
            
            if obs.must_go:
                # must-go event will be set again after receiving actions
                self.must_go.clear()

    def _setup_extractors_and_subs(self, mappings_config: dict):
        """
        Dynamically initializes MSG_EXTRACTORs and ROS 2 subscribers
        based on the dataset.yaml mappings.
        """
        # Ensure MSG_EXTRACTOR is imported at the top of your file:
        # from your_library.extractors import MSG_EXTRACTOR 

        for feature_key, mapping in mappings_config.items():
            # The mapping could be a single dict or a list of dicts (e.g., joint states)
            if not isinstance(mapping, list):
                mapping = [mapping]

            # Initialize storage for this feature. 
            # We use a list to maintain order if multiple topics map to one feature key.
            with self.latest_data_lock:
                if feature_key not in self.latest_data:
                    self.latest_data[feature_key] = [None] * len(mapping)
            
            self.expected_data_keys.add(feature_key)

            for idx, params in enumerate(mapping):
                # Copy params so .pop() doesn't mutate the original dictionary
                extractor_params = params.copy()
                
                msg_type_str = extractor_params.pop("msgtype")
                topic_name = extractor_params.pop("topic")
                
                # 1. Instantiate the corresponding purely mathematical Extractor
                if msg_type_str not in MSG_EXTRACTOR:
                    self.get_logger().error(f"No extractor registered for {msg_type_str}")
                    continue
                    
                extractor_instance = MSG_EXTRACTOR[msg_type_str](**extractor_params)

                # 2. Dynamically import the ROS 2 message class
                try:
                    parts = msg_type_str.split('/')
                    if len(parts) == 3:
                        pkg, _, cls_name = parts
                        module_name = f'{pkg}.msg'
                    else:
                        parts = msg_type_str.split('.')
                        cls_name = parts[-1]
                        module_name = '.'.join(parts[:-1])

                    msg_class = getattr(importlib.import_module(module_name), cls_name)
                except Exception as e:
                    self.get_logger().error(f'Failed to import {msg_type_str}: {e}')
                    continue

                # 3. Create the subscriber
                self.get_logger().info(f'Subscribing to {topic_name} -> {feature_key}[{idx}]')
                
                input_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5
                )
                
                # Note the lambda defaults (k=..., i=..., ext=...): 
                # This is required in Python to capture loop variables correctly!
                sub = self.create_subscription(
                    msg_class,
                    topic_name,
                    lambda msg, k=feature_key, i=idx, ext=extractor_instance: 
                        self._extractor_callback(msg, k, i, ext),
                    qos_profile=input_qos,
                    callback_group=self.callback_group,
                )
                self.subs.append(sub)

    def _extractor_callback(self, msg, feature_key: str, list_index: int, extractor):
        """
        Universal callback. Passes the raw ROS message to the purely mathematical
        extractor, then stores the resulting numpy array.
        """
        try:
            # The extractor returns a ready-to-use numpy array
            extracted_array = extractor.extract(msg)
            
            with self.latest_data_lock:
                # Store it in the correct position for this feature
                self.latest_data[feature_key][list_index] = extracted_array
                
        except Exception as e:
            self.get_logger().error(f"Error extracting {feature_key}: {e}")

    def _setup_processors_and_pubs(self, actions_config: list):
        """
        Setup action processsors and publishers depending on the action space.
        """
        for config in actions_config:
            processor_type = config["type"]
            if config.get('service'):
                # e.g. service: /franka_left/target_pose, srvtype: franka_custom_msgs/srv/SetPoseStamped
                cli, request_class = self._create_service_client(
                    config['service'], config.get('srvtype', 'franka_custom_msgs/srv/SetPoseStamped'))
                self.pubs.append((cli, request_class, config.get('request_field', 'pose')))
                self.action_processors.append(ACTION_PROCESSORS[processor_type](**config))
                continue
            msg_type_str = config['msgtype']
            topic_name = config['topic']
            parts = msg_type_str.split('/')
            if len(parts) == 3:
                pkg, _, cls_name = parts
                module_name = f'{pkg}.msg'
            else:
                parts = msg_type_str.split('.')
                cls_name = parts[-1]
                module_name = '.'.join(parts[:-1])
            msg_class = getattr(importlib.import_module(module_name), cls_name)
            
            self.pubs.append(self.create_publisher(msg_class, topic_name, 1))
            self.action_processors.append(ACTION_PROCESSORS[processor_type](**config))

    def _on_key_press(self, key):
        """Non-blocking callback triggered by pynput."""
        try:
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
                
                if char == 'r':
                    if not self.policy_active:
                        self.get_logger().info("▶️ STARTING Policy (R pressed)")
                        self.home_path = None
                        self.policy_active = True

                elif char == 's':
                    self.home_path = None
                    if self.policy_active:
                        self.get_logger().info("🛑 STOPPING Policy (S pressed)")
                        self.policy_active = False
                        
                elif char == 'h':
                    self.get_logger().info("🏠 Sending HOME command (H pressed)")
                    self.policy_active = False # Ensure policy stops executing
                    self._send_home_pose()
                    
        except Exception as e:
            self.get_logger().error(f"Keyboard listener error: {e}")

    def _pose_processor(self):
        """The first PoseAbsolute action processor that has sent a pose, or None."""
        for processor in self.action_processors:
            if getattr(processor, 'last_pose', None) is not None:
                return processor
        return None

    def _current_ee_pose(self):
        """Current pose of home_pose.ee_frame in home_pose.frame_id from TF, or None."""
        if self.tf_buffer is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.home_pose_cfg.get('frame_id', 'world'),
                                                self.home_pose_cfg['ee_frame'], rclpy.time.Time()).transform
        except Exception as e:
            self.get_logger().warn(f"No TF for {self.home_pose_cfg['ee_frame']}: {e}")
            return None
        return (np.array([t.translation.x, t.translation.y, t.translation.z]),
                np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))

    def _plan_home_path(self, pos, quat):
        """Rate-limited path from the last policy target (or the current pose) to the home pose."""
        processor = self._pose_processor()
        if processor is not None:
            start_pos, start_quat = processor.last_pose
        else:
            processor = next((p for p in self.action_processors if hasattr(p, 'make_msg')), None)
            start = self._current_ee_pose() if processor is not None else None
            if start is None:
                self.get_logger().warn("No pose target sent yet and no current pose from TF (home_pose.ee_frame), "
                                       "so the start of the home motion is unknown. Use the teleop home button.")
                return
            start_pos, start_quat = start
        rotations = Rotation.from_quat([start_quat, quat])
        distance = np.linalg.norm(np.asarray(pos) - start_pos)
        angle = (rotations[1] * rotations[0].inv()).magnitude()
        duration = max(distance / self.home_pose_cfg.get('max_linear_speed', 0.1),
                       angle / self.home_pose_cfg.get('max_angular_speed', 0.5))
        steps = max(1, math.ceil(duration * self.frequency))
        s = np.arange(1, steps + 1) / steps
        positions = start_pos + s[:, None] * (np.asarray(pos) - start_pos)
        quats = Slerp([0.0, 1.0], rotations)(s).as_quat()
        self.get_logger().info(f"Homing: {distance * 100:.1f} cm, {np.degrees(angle):.0f} deg in {duration:.1f} s")
        self.home_path = deque(zip(positions, quats))
        self._home_processor = processor

    def _home_step(self):
        path = self.home_path
        if not path:
            self.home_path = None
            return
        pos, quat = path.popleft()
        msg = self._home_processor.make_msg(pos, quat, self.get_clock().now().to_msg())
        self._send(self.home_cli, msg)

    def _send_home_pose(self):
        """Builds and publishes the Cartesian home position."""
        stamp = self.get_clock().now().to_msg()
        pos = self.home_pose_cfg.get('position', [0.3, 0.0, 0.5])
        quat = self.home_pose_cfg.get('orientation', [1.0, 0.0, 0.0, 0.0])  # x, y, z, w

        if self.home_cli is not None:
            self._plan_home_path(pos, quat)
        else:
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.home_pose_cfg.get('frame_id', 'world')
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos[0], pos[1], pos[2]
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat[0], quat[1], quat[2], quat[3]
            self.home_pub.publish(msg)

        gripper_msg = JointTrajectory()
        gripper_msg.header.stamp = stamp
        gripper_msg.joint_names = [self.home_pose_cfg.get('gripper_joint_name', 'franka_right_rh_r1')]
        
        point = JointTrajectoryPoint()
        # Grab the target value from YAML
        point.positions = [float(self.home_pose_cfg.get('gripper_open_val', 1.0))]
        
        # Set a safe execution duration (e.g., matching the 30Hz loop, or slightly longer for a smooth open)
        point.time_from_start.nanosec = int((1.0 / self.frequency) * 1e9)
        gripper_msg.points = [point]

        self.home_gripper_pub.publish(gripper_msg)

def main():
    parser = argparse.ArgumentParser(description="ROS client for a remote (LeRobot async-inference) policy server")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "robi_client.yaml"),
        help="client YAML (default: config/robi_client.yaml of this package)",
    )
    args, _ = parser.parse_known_args()
    rclpy.init()

    robi_client_node = RobiClient(config_file_path=args.config)
    
    executor = MultiThreadedExecutor()
    executor.add_node(robi_client_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robi_client_node.is_running = False
        robi_client_node.receive_thread.join()
        executor.shutdown()
        robi_client_node.destroy_node()


if __name__ == '__main__':
    main()