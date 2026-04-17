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
        
        self.lbl_name = QLabel(self.name)
        layout.addWidget(self.lbl_name)
        
        self.btn_action = QPushButton("Start")
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
            self.btn_action.setText("Start")
        elif state == "yellow":
            self.btn_action.setStyleSheet("background-color: yellow; color: black;")
            self.btn_action.setText("Starting...")
        elif state == "green":
            self.btn_action.setStyleSheet("background-color: green; color: white;")
            self.btn_action.setText("Stop")
        elif state == "red":
            self.btn_action.setStyleSheet("background-color: red; color: white;")
            self.btn_action.setText("Restart")

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
            ("Run Robot", ["pixi run -e humble ros2 launch franka_launch example.launch.py spawn_franka_left:=false spawn_franka_right:=true use_fake_hardware:=true"]),
            ("Run Teleoperation", [
                "pixi run -e humble ros2 launch franka_meta_quest start.launch.py", 
                "pixi run -e humble ros2 launch franka_data_collection start.launch.py"
            ]),
            ("Play Rosbag", ["pixi run -e humble ros2 bag play /home/scherer/frankapy/pick_and_place_duplo_mcap/episode_20260410_182328"])
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
        
        # Bottom Panel - Plots
        plot_scroll = QScrollArea()
        plot_scroll.setWidgetResizable(True)
        plot_panel = QWidget()
        self.plot_layout = QVBoxLayout(plot_panel)
        plot_scroll.setWidget(plot_panel)
        
        self.plots = {} # topic -> dict of widget, curves, data_x, data_y
        
        right_layout.addWidget(plot_scroll, stretch=1)
        main_layout.addWidget(right_panel)
        
        self.max_pts = 200

    def init_ros_connections(self):
        self.ros_interface.image_received.connect(self.update_image)
        self.ros_interface.joint_state_received.connect(self.update_plot)

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

    def update_plot(self, topic, pos, timestamp):
        num_joints = len(pos)
        
        if topic not in self.plots:
            widget = pg.PlotWidget(title=f"{topic}")
            widget.addLegend()
            widget.setMinimumHeight(200)
            self.plot_layout.addWidget(widget)
            curves = [widget.plot(pen=(i, num_joints), name=f"J{i}") for i in range(num_joints)]
            self.plots[topic] = {
                'widget': widget,
                'curves': curves,
                'data_x': [],
                'data_y': [[] for _ in range(num_joints)]
            }
            
        p = self.plots[topic]
        
        p['data_x'].append(timestamp)
        if len(p['data_x']) > self.max_pts:
            p['data_x'].pop(0)
            
        for i in range(num_joints):
            if i < len(p['data_y']):
                p['data_y'][i].append(pos[i])
                if len(p['data_y'][i]) > self.max_pts:
                    p['data_y'][i].pop(0)
                
                # Ensure sizes match before drawing to prevent crash
                if len(p['data_x']) == len(p['data_y'][i]):
                    p['curves'][i].setData(p['data_x'], p['data_y'][i])

    def closeEvent(self, event):
        self.ros_interface.stop()
        event.accept()
