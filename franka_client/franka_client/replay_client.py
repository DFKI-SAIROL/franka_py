import yaml
import threading
from queue import Queue
import time
import pickle
import importlib
import pathlib

import grpc
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pynput import keyboard
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import services_pb2
import services_pb2_grpc
from lerobot_utils import grpc_channel_options, send_bytes_in_chunks, get_logger
from action_processors import ACTION_PROCESSORS

# Import identical dataclasses to allow correct unpickling on the server
from franka_client.client import TimedObservation, TimedAction, Action

class ReplayClient(Node):
    def __init__(self, config_file_path: str):
        with open(config_file_path, 'r') as f:
            self.config = yaml.safe_load(f)

        node_name = self.config.get('node_name', 'replay_client')
        super().__init__(node_name)
        self.logger = get_logger(node_name)

        self.frequency = self.config.get('frequency', 30.0)
        
        self.pubs, self.action_processors = [], []
        self._setup_processors_and_pubs(self.config.get("actions", []))
        
        self.home_pose_cfg = self.config.get('home_pose', {})
        self.home_pub = self.create_publisher(PoseStamped, self.home_pose_cfg.get('topic'), 1)
        self.home_gripper_pub = self.create_publisher(
            JointTrajectory,
            self.home_pose_cfg.get('gripper_topic', '/franka_right/gripper/gripper_controller/joint_trajectory'),
            1
        )
        
        self.get_logger().info("Keyboard Controls: [R]un Replay | [S]top Replay | [H]ome Pose and Reset")
        self.listener = keyboard.Listener(on_press=self._on_key_press)
        self.listener.start()

        self.action_queue = Queue()
        self.action_queue_lock = threading.Lock()
        
        self.server_address = self.config.get('server_address', '127.0.0.1:8080')
        self.channel = grpc.insecure_channel(self.server_address, grpc_channel_options())
        self.stub = services_pb2_grpc.AsyncInferenceStub(self.channel)
        
        self.is_running = True
        
        dataset_cfg = self.config.get('dataset', {})
        replay_config = {
            "dataset_repo_id": dataset_cfg.get('repo_id', ''),
            "dataset_root": dataset_cfg.get('root', None),
            "episode_index": dataset_cfg.get('episode', 0),
            "fps": self.frequency,
            "actions_per_chunk": dataset_cfg.get('actions_per_chunk', 1),
            "lock_step": dataset_cfg.get('lock_step', True),
        }

        try:
            self.stub.Ready(services_pb2.Empty())
            self.logger.info("Connected to replay server!")
            
            config_bytes = pickle.dumps(replay_config)
            self.stub.SendPolicyInstructions(services_pb2.PolicySetup(data=config_bytes))
            self.logger.info("Sent dataset config to server.")
        except Exception as e:
            self.logger.error(f"Failed to connect or setup replay server: {e}")
            
        self.receive_thread = threading.Thread(target=self.receive_actions, daemon=True)
        self.receive_thread.start()

        timer_period = 1.0 / self.frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop_step)

    def _setup_processors_and_pubs(self, actions_config: list):
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

    def _send_command(self, cmd: str):
        obs = TimedObservation(
            timestamp=time.time(),
            timestep=0,
            observation={"command": cmd}
        )
        try:
            obs_bytes = pickle.dumps(obs)
            iterator = send_bytes_in_chunks(obs_bytes, services_pb2.Observation, silent=True)
            self.stub.SendObservations(iterator)
        except Exception as e:
            self.logger.error(f"Failed to send command {cmd}: {e}")

    def _on_key_press(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                char = key.char.lower()
                if char == 'r':
                    self.get_logger().info("▶️ PLAY Replay")
                    self._send_command("play")
                elif char == 's':
                    self.get_logger().info("🛑 STOP Replay")
                    self._send_command("stop")
                elif char == 'h':
                    self.get_logger().info("🏠 Sending HOME command")
                    self._send_command("stop")
                    self._send_home_pose()
                    self.get_logger().info("🔄 RESETTING Replay via Home")
                    self._send_command("reset")
                    with self.action_queue_lock:
                        while not self.action_queue.empty():
                            self.action_queue.get_nowait()
        except Exception as e:
            pass

    def receive_actions(self):
        self.logger.info("Action receiving thread starting")
        while self.is_running:
            try:
                actions_chunk = self.stub.GetActions(services_pb2.Empty())
                if len(actions_chunk.data) == 0:
                    time.sleep(1.0/self.frequency) 
                    continue 

                timed_actions = pickle.loads(actions_chunk.data)  # nosec
                
                with self.action_queue_lock:
                    for a in timed_actions:
                        self.action_queue.put(a)
                        
            except grpc.RpcError as e:
                time.sleep(1.0) 

    def execute_actions(self, action_array):
        stamp = self.get_clock().now().to_msg()
        for pub, processor in zip(self.pubs, self.action_processors):
            msg = processor.process(action_array, stamp)
            if msg is not None:
                pub.publish(msg)

    def control_loop_step(self):
        # Executes exactly one action in the queue each frame
        with self.action_queue_lock:
            if not self.action_queue.empty():
                timed_action = self.action_queue.get_nowait()
                self.execute_actions(timed_action.get_action())

    def _send_home_pose(self):
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
        point.positions = [float(self.home_pose_cfg.get('gripper_open_val', 1.0))]
        point.time_from_start.nanosec = int((1.0 / self.frequency) * 1e9)
        gripper_msg.points = [point]
        self.home_gripper_pub.publish(gripper_msg)

def main():
    rclpy.init()
    client_node = ReplayClient(
        config_file_path=pathlib.Path(__file__).parent / '../config/replay_client.yaml'
    )
    
    executor = MultiThreadedExecutor()
    executor.add_node(client_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        client_node.is_running = False
        client_node.receive_thread.join()
        executor.shutdown()
        client_node.destroy_node()

if __name__ == '__main__':
    main()
