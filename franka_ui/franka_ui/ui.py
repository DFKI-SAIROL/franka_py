import sys
import os
import yaml
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, 
    QScrollArea, QFrame, QSizePolicy, QGridLayout
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
import numpy as np
from .process_manager import ProcessManager

class NodeControlWidget(QFrame):
    def __init__(self, name, command, process_manager, parent=None):
        super().__init__(parent)
        self.name = name
        self.command = command
        self.pm = process_manager
        
        self.setFrameShape(QFrame.StyledPanel)
        layout = QHBoxLayout(self)
        
        self.btn_action = QPushButton(self.name)
        self.btn_action.clicked.connect(self.toggle_process)
        layout.addWidget(self.btn_action)
        
        self.btn_term = QPushButton("Terminal")
        self.btn_term.clicked.connect(self.open_terminal)
        layout.addWidget(self.btn_term)
        
        self.set_state("white")

    def set_state(self, state):
        self.state = state
        if state == "white":
            self.btn_action.setStyleSheet("background-color: white; color: black;")
        elif state == "yellow":
            self.btn_action.setStyleSheet("background-color: yellow; color: black;")
        elif state == "green":
            self.btn_action.setStyleSheet("background-color: green; color: white;")
        elif state == "red":
            self.btn_action.setStyleSheet("background-color: red; color: white;")

    def toggle_process(self):
        if self.state in ["white", "red"]:
            self.pm.start_process(self.name, self.command)
            self.set_state("yellow")
        elif self.state in ["yellow", "green"]:
            self.pm.stop_process(self.name)
            self.set_state("white")

    def open_terminal(self):
        self.pm.open_terminal(self.name)

    def check_status(self):
        if self.state == "white":
            return
            
        is_running = self.pm.is_running(self.name)
        if not is_running and self.state in ["yellow", "green"]:
            self.set_state("red")
        elif is_running and self.state == "yellow":
            # For now, transition to green if running. A true lifecycle node checks ROS state.
            self.set_state("green")


class FrankaUI(QWidget):
    def __init__(self, ros_interface):
        super().__init__()
        self.ros_interface = ros_interface
        self.pm = ProcessManager()
        
        self.init_ui()
        self.init_ros_connections()
        
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.check_all_statuses)
        self.status_timer.start(1000)

    def init_ui(self):
        self.setWindowTitle("Franka Control UI")
        self.resize(1200, 800)
        
        main_layout = QHBoxLayout(self)
        
        # Left Panel - Controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_panel.setFixedWidth(300)
        
        self.btn_master = QPushButton("Master Start")
        self.btn_master.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; font-size: 14px; padding: 10px;")
        self.btn_master.clicked.connect(self.master_start)
        left_layout.addWidget(self.btn_master)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        self.controls_layout = QVBoxLayout(scroll_content)
        self.controls_layout.setAlignment(Qt.AlignTop)
        
        # Add nodes
        self.node_widgets = []
        
        nodes = [
            ("Start Robot", [
                "pixi run -e humble ros2 launch franka_launch example.launch.py spawn_franka_left:=false spawn_franka_right:=true use_fake_hardware:=false ; exec bash",
                {"cmd": "pixi run -e humble ros2 launch franka_safety_layer start_ijk.launch.py spawn_franka_left:=false spawn_franka_right:=true bypass_safety:=true ; exec bash", "split": "-v"}
            ]),
            ("Start Video Stream", [
                {"cmd": "env -i HOME=$HOME USER=$USER /usr/bin/ssh -t jetson 'cd Projects/ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && bash launch_zed.sh csil'", "split": "-h"},
            ]),
            ("Run Teleoperation", [
                "pixi run -e humble ros2 launch franka_meta_quest start.launch.py", 
                "pixi run -e humble ros2 launch franka_data_collection start.launch.py"
            ])
        ]
        
        for name, cmd in nodes:
            w = NodeControlWidget(name, cmd, self.pm)
            self.node_widgets.append(w)
            self.controls_layout.addWidget(w)
            
        scroll.setWidget(scroll_content)
        left_layout.addWidget(scroll)
        main_layout.addWidget(left_panel)
        
        # Middle + Bottom Panel
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # Middle Panel - Video Feeds
        video_panel = QWidget()
        self.video_layout = QGridLayout(video_panel)
        self.video_labels = {} # topic -> QLabel
        
        # We need to create labels for expected topics
        try:
            with open(self.ros_interface.zed_config_path, 'r') as f:
                zed_cfg = yaml.safe_load(f)
                rgb_topics = zed_cfg['zed_rig_aggregator']['ros__parameters']['rgb_topics']
                for i, topic in enumerate(rgb_topics):
                    lbl = QLabel("No Video: " + topic)
                    lbl.setAlignment(Qt.AlignCenter)
                    lbl.setStyleSheet("background-color: black; color: white;")
                    lbl.setMinimumSize(320, 240)
                    self.video_layout.addWidget(lbl, 0, i)
                    self.video_labels[topic] = lbl
        except Exception:
            pass
            
        right_layout.addWidget(video_panel, stretch=2)

        self.lbl_recording_status = QLabel(" Recording: No ")
        self.lbl_recording_status.setStyleSheet("background-color: gray; color: white; font-weight: bold; font-size: 14px; padding: 5px;")
        self.lbl_recording_status.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.lbl_recording_status)
        
        # Bottom Panel - Plots
        plot_scroll = QScrollArea()
        plot_scroll.setWidgetResizable(True)
        plot_panel = QWidget()
        self.plot_layout = QGridLayout(plot_panel)
        plot_scroll.setWidget(plot_panel)
        
        self.plots = {} # topic -> dict of widget, curves, data_x, data_y
        
        right_layout.addWidget(plot_scroll, stretch=1)
        main_layout.addWidget(right_panel)
        
        self.max_pts = 200
        self.has_recorded = False
        self.is_recording = False

    def init_ros_connections(self):
        self.ros_interface.image_received.connect(self.update_image)
        self.ros_interface.joint_state_received.connect(self.update_plot)
        if hasattr(self.ros_interface, 'reset_plots_received'):
            self.ros_interface.reset_plots_received.connect(self.clear_plots)
        if hasattr(self.ros_interface, 'recording_status_received'):
            self.ros_interface.recording_status_received.connect(self.update_recording_status)

    def master_start(self):
        for w in self.node_widgets:
            if w.state in ["white", "red"]:
                w.toggle_process()

    def check_all_statuses(self):
        for w in self.node_widgets:
            w.check_status()

    def update_image(self, topic, cv_image):
        if topic in self.video_labels:
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qimg = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            # Scale keeping aspect ratio
            scaled_pixmap = pixmap.scaled(self.video_labels[topic].size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_labels[topic].setPixmap(scaled_pixmap)
            
    def clear_plots(self):
        for topic, p_dict in self.plots.items():
            p_dict['data_x'].clear()
            for dim_data in p_dict['data_y']:
                dim_data.clear()
            for curve in p_dict['curves']:
                curve.setData([], [])
            p_dict['start_time'] = None

    def update_recording_status(self, is_recording):
        self.is_recording = is_recording
        if is_recording:
            self.has_recorded = True
            self.lbl_recording_status.setText(" Recording: Yes ")
            self.lbl_recording_status.setStyleSheet("background-color: green; color: white; font-weight: bold; font-size: 14px; padding: 5px;")
            self.clear_plots()
        else:
            self.lbl_recording_status.setText(" Recording: No ")
            self.lbl_recording_status.setStyleSheet("background-color: gray; color: white; font-weight: bold; font-size: 14px; padding: 5px;")

    def update_plot(self, topic, pos, timestamp):
        if self.has_recorded and not self.is_recording:
            return
            
        num_joints = len(pos)
        
        if topic not in self.plots:
            self.plots[topic] = {
                'widgets': [],
                'curves': [],
                'data_x': [],
                'data_y': [[] for _ in range(num_joints)],
                'last_gui_update_time': 0.0,
                'start_time': timestamp
            }
            # Create a separate widget for each joint
            p = self.plots[topic]
            for i in range(num_joints):
                widget = pg.PlotWidget(title=f"{topic} - Dim {i}")
                widget.setMinimumHeight(120)
                
                fr3_limits = [
                    (-2.3093, 2.3093),
                    (-1.5133, 1.5133),
                    (-2.4937, 2.4937),
                    (-2.7478, -0.4461),
                    (-2.4800, 2.4800),
                    (0.8521, 4.2094),
                    (-2.6895, 2.6895)
                ]
                if i < len(fr3_limits):
                    widget.setYRange(fr3_limits[i][0], fr3_limits[i][1])
                
                row = i // 4
                col = i % 4
                self.plot_layout.addWidget(widget, row, col)
                curve = widget.plot(pen=(i, num_joints), name=f"Dim {i}")
                p['widgets'].append(widget)
                p['curves'].append(curve)
                
        p = self.plots[topic]
        if p.get('start_time') is None:
            p['start_time'] = timestamp
        
        p['data_x'].append(timestamp)
        for i in range(num_joints):
            if i < len(p['data_y']):
                p['data_y'][i].append(pos[i])
                
        # Enforce 20s window based on timestamps
        while len(p['data_x']) > 0 and (timestamp - p['data_x'][0]) > 20.0:
            p['data_x'].pop(0)
            for i in range(num_joints):
                if i < len(p['data_y']):
                    p['data_y'][i].pop(0)
                
        # Update curve data at 1Hz max to save resources
        if timestamp - p.get('last_gui_update_time', 0.0) >= 1.0:
            p['last_gui_update_time'] = timestamp
            display_x = [t - p.get('start_time', timestamp) for t in p['data_x']]
            for i in range(num_joints):
                if i < len(p['data_y']):
                    if len(p['data_x']) == len(p['data_y'][i]):
                        p['curves'][i].setData(display_x, p['data_y'][i])

    def closeEvent(self, event):
        self.pm.kill_session()
        self.ros_interface.stop()
        event.accept()
