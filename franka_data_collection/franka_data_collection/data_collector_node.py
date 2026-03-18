import os
import signal
import subprocess
from datetime import datetime
from pathlib import Path
import yaml

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'data_collector.yaml')

        if not config_file:
            self.get_logger().error('No config_file parameter provided!')
            return

        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        # 1. Configuration Setup
        self.storage_path = Path(self.config.get('storage_path', './data'))

        self.storage_path.mkdir(parents=True, exist_ok=True)
        
        # Assuming your YAML parser just returns a flat list of topic names now
        self.topics_to_record = self.config.get('topics', None)
        if self.topics_to_record is None:
            self.get_logger().error('No topics to record provided!')
            return

        # 2. State Variables
        self.is_recording = False
        self.record_process = None
        self.episode_path = None

        # 3. Services
        self.srv_trigger = self.create_service(
            Trigger, '~/record_data_trigger', self.recording_callback
        )

        self.get_logger().info('Data collector ready. Waiting for trigger...')

    def recording_callback(self, request, response):
        """Toggles the recording state."""
        if not self.is_recording:
            success, message = self.start_recording()
        else:
            success, message = self.stop_recording()

        response.success = success
        response.message = message
        return response

    def start_recording(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.episode_path = self.storage_path / f'episode_{timestamp}'
        
        # Build the ros2 bag record command
        command = [
            'ros2', 'bag', 'record',
            '-o', str(self.episode_path),
            '-s', 'mcap',  # Explicitly use MCAP format
        ] + self.topics_to_record

        self.get_logger().info(f'Starting recording to: {self.episode_path}')
        self.get_logger().info(f"Command: {' '.join(command)}")

        try:
            # Launch the bagger as a completely separate subprocess
            self.record_process = subprocess.Popen(command)
            self.is_recording = True
            return True, f"Started recording to {self.episode_path}"
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")
            return False, str(e)

    def stop_recording(self):
        if self.record_process is None:
            return False, "No active recording process found."

        self.get_logger().info("Stopping recording and saving MCAP file...")

        # CRITICAL: Send SIGINT (Ctrl+C) to gracefully stop the bagger. 
        # If you use .kill() or SIGTERM, the MCAP file might not write its metadata/index.
        self.record_process.send_signal(signal.SIGINT)
        
        # Wait for the process to finish writing to disk and close
        self.record_process.wait()
        self.record_process = None
        self.is_recording = False

        self.get_logger().info("Recording saved successfully.")
        return True, "Stopped recording and saved data."


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Catch Ctrl+C to ensure we don't leave a dangling rosbag process
        if node.is_recording:
            node.stop_recording()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()