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

try:
    from .handlers import get_handler
except:
    from franka_data_collection.handlers import get_handler

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
        
        # Create storage directory for this episode
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.episode_path = self.storage_path / f'episode_{timestamp}'
        self.episode_path.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Logging to: {self.episode_path}')

        # Storage for latest messages: {logical_name: msg}
        self.latest_msgs = {}
        self.latest_msgs_lock = threading.Lock()
        
        # Buffers for saving: {logical_name: [list_of_data]}
        self.data_buffers = {}
        
        # Handlers: {logical_name: handler_instance}
        self.handlers = {}

        # Setup subscribers
        self.subs = []
        self._setup_topics(self.config.get('topics', {}))

        # Setup timer for sampling
        self.timer = self.create_timer(1.0 / self.logging_rate, self.sampling_callback)
        self.get_logger().info(f'Data collector started at {self.logging_rate} Hz')

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
        
        # Set up handler
        handler = get_handler(msg_type_str, fields)
        if handler:
            self.handlers[logical_name] = handler
        else:
            self.get_logger().warn(f'No specific handler for {msg_type_str}, saving raw objects (might fail pickle)')
            self.handlers[logical_name] = None

        sub = self.create_subscription(
            msg_class,
            topic_name,
            lambda msg, name=logical_name: self.topic_callback(msg, name),
            10
        )
        self.subs.append(sub)

    def topic_callback(self, msg, logical_name):
        with self.latest_msgs_lock:
            self.latest_msgs[logical_name] = msg

    def sampling_callback(self):
        # Snapshot the latest messages
        with self.latest_msgs_lock:
            snapshot = self.latest_msgs.copy()
        for name, buffer_list in self.data_buffers.items():
            if name in snapshot:
                msg = snapshot[name]
                handler = self.handlers.get(name)
                
                if handler:
                    # Extract structured data
                    try:
                        data = handler.extract(msg)
                        buffer_list.append(data)
                    except Exception as e:
                        # self.get_logger().warn(f"Extraction failed for {name}: {e}")
                        buffer_list.append(None)
                else:
                    # Fallback to raw object
                    buffer_list.append(msg)
            else:
                buffer_list.append(None)

        # Flush every N steps 
        if len(list(self.data_buffers.values())[0]) >= 300:
            print("Flushing data to disk...")
        #     self.flush_to_disk()

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
                            # Attempt to stack
                            stacked[k] = np.array(vals)
                        except:
                            # Fallback to object array
                            stacked[k] = np.array(vals, dtype=object)
                    
                    np.save(save_path, stacked, allow_pickle=True)
                else:
                    # Fallback
                    np.save(save_path, np.array(buffer_list, dtype=object), allow_pickle=True)
            except Exception as e:
                self.get_logger().error(f'Failed to save {name}: {e}')
            
            # Clear buffer
            buffer_list.clear()

    # def destroy_node(self):
    #     # Flush remaining
    #     self.flush_to_disk()
    #     super().destroy_node()

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
