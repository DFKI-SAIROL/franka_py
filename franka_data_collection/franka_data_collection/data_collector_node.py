import functools
import importlib
import os
import time
import threading
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import yaml
import numpy as np
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

# Import message types for _process_msg
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from custom_interfaces.msg import RigSnapshot
from franka_custom_msgs.msg import FMQDebug

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# Helper to get nested recursive dict
def recursive_dict_update(d, u):
    for k, v in u.items():
        if isinstance(v, dict):
            d[k] = recursive_dict_update(d.get(k, {}), v)
        else:
            d[k] = v
    return d

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # Declare parameter for config file
        # self.declare_parameter('config_file', '')
        # config_file = self.get_parameter('config_file').get_parameter_value().string_value

        # For now, we hardcode the config file path as requested
        config_file = '/home/csil/projects/bimanual/franka_ros2_ws/src/frankapy/franka_data_collection/config/data_collector.yaml'
        
        if not config_file:
            self.get_logger().error('No config_file parameter provided!')
            return

        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        self.logging_rate = self.config.get('logging_rate', 30.0)
        self.storage_path = Path(self.config.get('storage_path', './data'))
        
        # State
        self.is_recording = False
        self.episode_path = None
        self.teleop_input = False
        self.teleop_active = {}
        
        # Trigger config
        self.trigger_topic_name = "target_cartesian_pose" 
        self.has_trigger_topic = False
        self.trigger_logical_name = None 

        self.get_logger().info(f'Data collector ready. Service based recording.')

        # Storage for latest PROCESSED data: {logical_name: data_dict}
        self.latest_data = {}
        self.latest_data_lock = threading.Lock()
        
        # Buffers for saving: {logical_name: [list_of_data]}
        self.data_buffers = {}
        
        # Config for fields extraction: {logical_name: [fields]}
        self.topic_fields = {}

        # Setup subscribers
        self.subs = []
        self._setup_topics(self.config.get('topics', {}))
        
        # Setup Services
        self.srv_start = self.create_service(Trigger, '~/record_data_trigger', self.recording_callback)
        # self.srv_stop = self.create_service(Trigger, '~/stop_recording', self.stop_recording_callback)

        # Scan for trigger
        self._scan_for_trigger(self.config.get('topics', {}))

        self.bridge = CvBridge()
        
        if self.has_trigger_topic:
            self.get_logger().info(f"Event-driven mode: Triggered by *{self.trigger_topic_name} (logical name: {self.trigger_logical_name})")
            # Do NOT create timer
        else:
            # Fallback to timer
            self.timer = self.create_timer(1.0 / self.logging_rate, self.sampling_callback)
            self.get_logger().info(f'Data collector timer running at {self.logging_rate} Hz')

    def start_recording(self):

        # Create storage directory for this episode
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.episode_path = self.storage_path / f'episode_{timestamp}'
        self.episode_path.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Started recording to: {self.episode_path}')
        
        # Clear buffers just in case
        for name in self.data_buffers:
            self.data_buffers[name] = []
            
        self.is_recording = True
        return True, f"Started recording to {self.episode_path}"

    def stop_recording(self):           
        self.is_recording = False
        self.get_logger().info("Stopped recording. Flushing data...")
        self.flush_to_disk()
        return True, "Stopped recording and saved data."

    def recording_callback(self, request, response):
        if not self.is_recording:
            success, message = self.start_recording()
        else:
            success, message = self.stop_recording()

        response.success = success
        response.message = message
        return response

    # def stop_recording_callback(self, request, response):
    #     success, message = self.stop_recording()
    #     response.success = success
    #     response.message = message
    #     return response

    def _scan_for_trigger(self, topics_config, parent_logical_name=None):
        for logical_name, items in topics_config.items():
            current_logical_name = logical_name # if parent_logical_name is None else f"{parent_logical_name}.{logical_name}"
            if isinstance(items, dict):
                 if 'topic' in items:
                     if items['topic'].endswith(self.trigger_topic_name):
                         self.has_trigger_topic = True
                         self.trigger_logical_name = current_logical_name
                         return # Found it, no need to search further
                 else:
                     self._scan_for_trigger(items, current_logical_name)
                     if self.has_trigger_topic: # If found in recursion, propagate stop
                         return

    def _setup_topics(self, topics_config):
        """Recursively flush out topics configuration"""
        for category, items in topics_config.items():
            if not isinstance(items, dict):
                continue
                
            # Check if this item is a topic definition (has 'topic' and 'msg_type')
            if 'topic' in items and 'msg_type' in items:
                self._create_subscriber(category, items)
            else:
                # Recurse
                for key, val in items.items():
                    if isinstance(val, dict) and 'topic' in val:
                         self._create_subscriber(key, val)

    def _create_subscriber(self, logical_name, cfg):
        topic_name = cfg['topic']
        msg_type_str = cfg['msg_type']
        fields = cfg.get('fields', None)
        
        if fields:
            self.topic_fields[logical_name] = fields

        # Import message type
        try:
            parts = msg_type_str.split('/')
            if len(parts) == 3:
                pkg, _, cls_name = parts
                module_name = f'{pkg}.msg'
            else:
                 parts = msg_type_str.split('.')
                 cls_name = parts[-1]
                 module_name = '.'.join(parts[:-1])

            module = importlib.import_module(module_name)
            msg_class = getattr(module, cls_name)
        except Exception as e:
            self.get_logger().error(f'Failed to import message type {msg_type_str}: {e}')
            return

        self.get_logger().info(f'Subscribing to {logical_name} ({topic_name}) as {msg_type_str}')
        
        # Initialize buffer list
        self.data_buffers[logical_name] = []
        
        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        sub = self.create_subscription(
            msg_class,
            topic_name,
            lambda msg, name=logical_name: self._topic_callback(msg, name),
            qos_profile=input_qos
        )
        self.subs.append(sub)

    def _topic_callback(self, msg, logical_name):
        # Dispatch processing (and potentially triggering)
        self._process_msg(msg, logical_name)

    @functools.singledispatchmethod
    def _process_msg(self, msg, logical_name):
        """Base handler for unknown message types"""
        with self.latest_data_lock:
            self.latest_data[logical_name] = msg

    @_process_msg.register
    def _(self, msg: JointState, logical_name):
        data = {}
        fields = self.topic_fields.get(logical_name, ['position', 'velocity', 'effort'])
        for field in fields:
            if hasattr(msg, field):
                data[field] = np.array(getattr(msg, field), dtype=np.float64)
        
        with self.latest_data_lock:
            self.latest_data[logical_name] = data

    @_process_msg.register
    def _(self, msg: PoseStamped, logical_name):
        data = {}
        fields = self.topic_fields.get(logical_name, ['position', 'orientation'])
        
        if 'position' in fields:
            data['position'] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float64)
        if 'orientation' in fields:
            data['orientation'] = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=np.float64)
        
        with self.latest_data_lock:
            self.latest_data[logical_name] = data

    @_process_msg.register
    def _(self, msg: JointTrajectory, logical_name):
        data = {}
        fields = self.topic_fields.get(logical_name, ['position', 'velocity'])
        
        # Gripper controllers usually send 1 point with the target positions
        if len(msg.points) > 0:
            if 'position' in fields and msg.points[0].positions:
                data['position'] = np.array(msg.points[0].positions, dtype=np.float64)
            if 'velocity' in fields and msg.points[0].velocities:
                data['velocity'] = np.array(msg.points[0].velocities, dtype=np.float64)
            
        with self.latest_data_lock:
            self.latest_data[logical_name] = data

    @_process_msg.register
    def _(self, msg: Image, logical_name):
        # Basic Image extraction
        dtype = np.uint8
        if '16UC1' in msg.encoding:
            dtype = np.uint16
        elif '32FC1' in msg.encoding:
            dtype = np.float32
            
        arr = np.frombuffer(msg.data, dtype=dtype)
        
        try:
            # Try simple reshape if possible, else return flat
            channels = 1
            if 'rgb' in msg.encoding or 'bgr' in msg.encoding:
                channels = 3
            elif 'rgba' in msg.encoding or 'bgra' in msg.encoding:
                channels = 4
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            arr = np.array(arr) 
            # arr = arr.reshape((msg.height, msg.width, channels))
        except:
             pass
             
        with self.latest_data_lock:
            self.latest_data[logical_name] = {'image': arr}
    
    @_process_msg.register
    def _(self, msg: RigSnapshot, logical_name):
        cam_names = msg.camera_names
        rgbs = msg.rgbs 

        data = {}
        fields = self.topic_fields.get(logical_name, None)

        for name, rgb in zip(cam_names, rgbs):
            if fields is not None and name not in fields:
                continue
            try:
                arr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
                data[name] = np.array(arr)
            except Exception as e:
                self.get_logger().error(f"Failed to process image {name} in RigSnapshot: {e}")

        with self.latest_data_lock:
            self.latest_data[logical_name] = data

    @_process_msg.register
    def _(self, msg: FMQDebug, logical_name):
        self.teleop_active[logical_name] = msg.movement_enabled
        data = {}
        fields = self.topic_fields.get(logical_name, ['delta_position', 'delta_orientation'])
        
        if 'delta_position' in fields:
            data['delta_position'] = np.array([msg.delta_target_pose.position.x, msg.delta_target_pose.position.y, msg.delta_target_pose.position.z])
            
        if 'delta_orientation' in fields:
            data['delta_orientation'] = np.array([msg.delta_target_pose.orientation.x, msg.delta_target_pose.orientation.y, msg.delta_target_pose.orientation.z, msg.delta_target_pose.orientation.w])
            
        with self.latest_data_lock:
            self.latest_data[logical_name] = data

    def sampling_callback(self):
        if not self.is_recording:
            return

        # Pure state-driven gating: stop appending data to avoid logging endless zeros in standstill.
        # This properly supports bimanual arrays because it pauses only if NO controllers are moving.
        if self.teleop_active and not any(self.teleop_active.values()):
            return

        # Snapshot the latest data
        with self.latest_data_lock:
            snapshot = self.latest_data.copy()
            
        for name, buffer_list in self.data_buffers.items():
            if name in snapshot:
                data = snapshot[name]
                buffer_list.append(data)
            else:
                buffer_list.append(None)

    def flush_to_disk(self):
        self.get_logger().info('Flushing data to disk...')
        for name, buffer_list in self.data_buffers.items():
            if not buffer_list:
                continue
            
            chunk_id = len(list(self.episode_path.glob(f'{name}_*.npy'))) 
            save_path = self.episode_path / f'{name}_{chunk_id:04d}.npy'
            
            try:
                # Peek first element to see if it is a dict
                valid_items = [x for x in buffer_list if x is not None]
                if valid_items and isinstance(valid_items[0], dict):
                    # Stack dict of arrays
                    stacked = {}
                    keys = valid_items[0].keys()
                    for k in keys:
                        # Collect all values for this key
                        vals = [item[k] if item is not None else np.nan for item in buffer_list] 
                        try:
                            # Try standard stacking
                            stacked[k] = np.stack(vals)
                        except:
                            # Fallback if dimensions vary (like variable length trajectory points)
                            stacked[k] = np.array(vals, dtype=object)
                    
                    np.save(save_path, stacked)
                else:
                    # Fallback
                    np.save(save_path, np.array(buffer_list, dtype=object))
            except Exception as e:
                self.get_logger().error(f'Failed to save {name}: {e}')
            
            # Clear buffer
            buffer_list.clear()
        self.get_logger().info('Finished saving data to disk...')

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
