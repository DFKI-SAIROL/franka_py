import importlib
import yaml
import threading
from queue import Queue
import time
import pickle
from dataclasses import dataclass

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

        # Storage for latest PROCESSED data: {logical_name: [arrays]}
        self.latest_data = {}
        self.latest_data_lock = threading.Lock()
        self.expected_data_keys = set()

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
        self.action_chunk_size = -1
        self._chunk_size_threshold = self.config.get('policy.chunk_size_threshold', 0.5)

        # gRPC Setup
        self.server_address = self.config.get('server_address')
        self.channel = grpc.insecure_channel(self.server_address, grpc_channel_options())
        self.stub = services_pb2_grpc.AsyncInferenceStub(self.channel)
        
        # start client and handshake with server
        yaml_policy_config = self.config.get('policy', {})
        self.policy_config = RemotePolicyConfig(
            yaml_policy_config.get('policy_type'),
            yaml_policy_config.get('pretrained_name_or_path'),
            yaml_policy_config.get('lerobot_features'),
            yaml_policy_config.get('actions_per_chunk'),
            yaml_policy_config.get('policy_device'),
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
                self.action_chunk_size = max(self.action_chunk_size, len(timed_actions))

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
        with self.action_queue_lock:
            # Avoid division by zero if chunk size isn't set yet
            chunk_size = max(self.action_chunk_size, 1)
            is_queue_low = self.action_queue.qsize() / chunk_size <= self._chunk_size_threshold

        if not is_queue_low:
            return False

        # 2. Check for ALL Essential Sensor Data (No Nones in lists)
        with self.latest_data_lock:
            for feature_key, array_list in self.latest_data.items():
                if any(v is None for v in array_list):
                    return False

        return True
    
    def build_observation(self, task: str = "Pick and Place") -> TimedObservation:
        """Assembles the observation automatically based on the YAML mappings"""
        raw_observation: RawObservation = {}
        
        with self.latest_data_lock:
            for feature_key in self.expected_data_keys:
                # self.latest_data[feature_key] is a list of arrays
                arrays = self.latest_data[feature_key]
                
                # If we are mapping multiple topics to one key (e.g., arm + gripper),
                # concatenate them. Otherwise, just grab the single array.
                if len(arrays) == 1:
                    raw_observation[feature_key] = arrays[0]
                else:
                    raw_observation[feature_key] = np.concatenate(arrays, axis=-1).astype(np.float32)
        
        raw_observation["task"] = task

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
                pub.publish(msg)

    def control_loop_step(self):
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
                        self.policy_active = True
                        
                elif char == 's':
                    if self.policy_active:
                        self.get_logger().info("🛑 STOPPING Policy (S pressed)")
                        self.policy_active = False
                        
                elif char == 'h':
                    self.get_logger().info("🏠 Sending HOME command (H pressed)")
                    self.policy_active = False # Ensure policy stops executing
                    self._send_home_pose()
                    
        except Exception as e:
            self.get_logger().error(f"Keyboard listener error: {e}")

    def _send_home_pose(self):
        """Builds and publishes the Cartesian home position."""
        stamp = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.home_pose_cfg.get('frame_id', 'world')
        
        pos = self.home_pose_cfg.get('position', [0.3, 0.0, 0.5])
        quat = self.home_pose_cfg.get('orientation', [1.0, 0.0, 0.0, 0.0])
        
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
    rclpy.init()

    robi_client_node = RobiClient(
        config_file_path="/home/csil/projects/bimanual/franka_ros2_ws/src/frankapy/franka_client/config/robi_client.yaml", 
    )
    
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