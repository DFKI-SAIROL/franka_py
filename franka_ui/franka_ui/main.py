import sys
import os
import signal
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
from .ros_interface import ROSInterface
from .ui import FrankaUI

def main(args=None):
    app = QApplication(sys.argv)
    
    # Define paths to config files
    workspace_dir = os.path.expanduser("~/projects/frankapy")
    zed_config_path = os.path.join(workspace_dir, "zed_rig_aggregator_node/config/zed_rig_aggregator_node_config.yaml")
    data_config_path = os.path.join(workspace_dir, "franka_data_collection/config/data_collector.yaml")
    
    # Initialize ROS interface in background thread
    ros_interface = ROSInterface(zed_config_path, data_config_path)
    ros_interface.start()
    
    # Initialize UI
    window = FrankaUI(ros_interface)
    window.show()
    
    # Execute App
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
