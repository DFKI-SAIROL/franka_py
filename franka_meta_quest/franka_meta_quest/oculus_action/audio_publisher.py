#!/usr/import/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import subprocess
import threading
import sys

class MetaQuestAudioPublisher(Node):
    def __init__(self):
        super().__init__('meta_quest_audio_publisher')
        
        # Publish audio as raw bytes to be saved in rosbags
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'audio', 10)
        
        self.get_logger().info('Starting scrcpy to capture Meta Quest microphone...')
        
        # Start scrcpy capturing microphone as raw 16-bit PCM
        # --no-video: Don't stream video
        # --audio-source=mic: Capture from headset microphone
        # --audio-codec=raw: Output raw PCM
        # --audio-output-mode=raw: Send raw output to stdout instead of playing it
        try:
            self.scrcpy_process = subprocess.Popen(
                ["scrcpy", "--no-video", "--audio-source=mic", "--audio-codec=raw", "--audio-format=s16le", "--audio-output-mode=raw"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=False
            )
            
            # Start a thread to read stdout and publish
            self.running = True
            self.read_thread = threading.Thread(target=self._read_and_publish_audio)
            self.read_thread.start()
            
            # Start a thread to monitor stderr
            self.err_thread = threading.Thread(target=self._monitor_stderr)
            self.err_thread.start()
        except FileNotFoundError:
            self.get_logger().error("scrcpy is not installed! Please run 'sudo apt install scrcpy' on your Ubuntu PC.")
            self.running = False

    def _monitor_stderr(self):
        while self.running and rclpy.ok() and self.scrcpy_process.stderr:
            try:
                err_line = self.scrcpy_process.stderr.readline()
                if not err_line:
                    break
                decoded = err_line.decode('utf-8', errors='replace').strip()
                if decoded:
                    self.get_logger().error(f"scrcpy: {decoded}")
            except Exception:
                break
                
    def _read_and_publish_audio(self):
        # 4096 bytes per chunk is a good balance of latency and overhead
        chunk_size = 4096 
        
        while self.running and rclpy.ok():
            try:
                # Read raw PCM data from scrcpy stdout
                data = self.scrcpy_process.stdout.read(chunk_size)
                if not data:
                    break
                
                # Publish the raw audio bytes
                msg = UInt8MultiArray()
                msg.data = list(data)
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error reading audio: {e}")
                break
                
        self.get_logger().info("Audio capture stopped.")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'scrcpy_process'):
            self.scrcpy_process.terminate()
            self.scrcpy_process.wait()
        if hasattr(self, 'read_thread'):
            self.read_thread.join()
        if hasattr(self, 'err_thread'):
            self.err_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MetaQuestAudioPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
