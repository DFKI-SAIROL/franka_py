import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtCore import QThread, pyqtSignal
from sensor_msgs.msg import Image, JointState
import yaml
from cv_bridge import CvBridge
import numpy as np
from lifecycle_msgs.msg import TransitionEvent
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ROSInterface(QThread):
    # Signals to communicate with GUI safely
    image_received = pyqtSignal(str, object) # topic_name, qimage (or np.array)
    joint_state_received = pyqtSignal(str, object, object) # topic_name, positions, timestamps
    lifecycle_event_received = pyqtSignal(str, str) # node_name, state
    reset_plots_received = pyqtSignal()
    recording_status_received = pyqtSignal(bool)

    def __init__(self, zed_config_path, data_config_path):
        super().__init__()
        self.zed_config_path = zed_config_path
        self.data_config_path = data_config_path
        self.bridge = CvBridge()
        self.running = True

    def run(self):
        rclpy.init()
        self.node = rclpy.create_node('franka_ui_node')

        # Read configs
        rgb_topics = []
        try:
            with open(self.zed_config_path, 'r') as f:
                zed_cfg = yaml.safe_load(f)
                rgb_topics = zed_cfg['zed_rig_aggregator']['ros__parameters']['rgb_topics']
        except Exception as e:
            self.node.get_logger().error(f"Failed to load ZED config: {e}")

        joint_topics = []
        try:
            with open(self.data_config_path, 'r') as f:
                data_cfg = yaml.safe_load(f)
                topics = data_cfg.get('topics', [])
                joint_topics = [t for t in topics if 'joint_states' in t]
        except Exception as e:
            self.node.get_logger().error(f"Failed to load Data config: {e}")

        # Subscriptions
        self.subs = []

        input_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )
        
        # Image subs
        for topic in rgb_topics:
            def make_image_cb(t):
                return lambda msg: self.image_cb(msg, t)
            sub = self.node.create_subscription(Image, topic, make_image_cb(topic), qos_profile=input_qos)
            self.subs.append(sub)

        # Joint subs
        for topic in joint_topics:
            def make_joint_cb(t):
                return lambda msg: self.joint_cb(msg, t)
            sub = self.node.create_subscription(JointState, topic, make_joint_cb(topic), 10)
            self.subs.append(sub)

        # Reset plots sub
        self.reset_trigger = self.node.create_service(Trigger, '/franka_meta_quest/reset_home', self.reset_cb)
        
        self.is_recording = False
        self.srv_trigger = self.node.create_service(Trigger, '/data_collector/record_data_trigger', self.recording_callback)

        # Lifecycle events (global topic usually)
        # Note: lifecycle state changes might come per node as ~transition_event or /rosout. 
        # Typically: /node_name/transition_event. We can listen to a known list or all.
        # But for now, we'll try to guess node names from command or use polling in UI instead.
        
        executor = MultiThreadedExecutor()
        executor.add_node(self.node)

        while self.running and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

        self.node.destroy_node()
        rclpy.shutdown()

    def image_cb(self, msg, topic):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.image_received.emit(topic, cv_image)
        except Exception as e:
            self.node.get_logger().error(f"CV Bridge error: {e}")

    def joint_cb(self, msg, topic):
        pos = np.array(msg.position)
        # Just use simple time or host time
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.joint_state_received.emit(topic, pos, t)

    def reset_cb(self, msg):
        self.reset_plots_received.emit()
        
    def recording_callback(self, request, response):
        self.is_recording = not self.is_recording
        self.recording_status_received.emit(self.is_recording)
        self.reset_plots_received.emit()
        response.success = True
        response.message = f"UI tracked recording state: {self.is_recording}"
        return response

    def stop(self):
        self.running = False
        self.wait()
