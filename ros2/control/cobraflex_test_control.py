#!/usr/bin/env python3
# CobraFlex Test Control -- Thesis Handover
# Source lineage: cobraflex_test_control_v14.py
# Handover cleanup: stable filename and English-only user-facing text.
# 2026-07-26 /clock-watchdog fix:
#   - classify clock health only from /clock callbacks, never from vehicle
#     motion, odometry, wheel speed, or pose changes;
#   - distinguish a frozen clock value from a temporarily delayed local
#     callback;
#   - tolerate a simulation reset by establishing a new timing baseline;
#   - include clock counters and ages in any watchdog error.
# 2026-07-20 recording-coverage fix:
#   - wait for rosbag2 subscriptions to appear in the ROS graph before the
#     pre-roll starts;
#   - abort the run if recorder discovery does not complete, rather than
#     producing a partial bag that looks formal;
#   - record a full 2.0 profile-clock seconds of zero command before motion;
#   - always include /clock in simulation-timed bags.
import sys
import math
import time
import threading
import subprocess
import signal
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                             QComboBox, QListWidget, QListWidgetItem, QGroupBox, 
                             QMessageBox, QFormLayout)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

SIM_ODOM_TRUTH_TOPIC = "/odom_truth"
SIM_CLOCK_TOPIC = "/clock"

# Recorder discovery is DDS/wall-time work, so its timeout must use
# monotonic wall time even when the motion profile itself uses /clock.
RECORDER_READY_TIMEOUT_WALL_SEC = 10.0
RECORDER_READY_CONFIRM_WALL_SEC = 0.25
RECORDER_GRAPH_POLL_WALL_SEC = 0.05
RECORDER_PREROLL_PROFILE_SEC = 2.0

# A publisher that keeps sending the same /clock value is a genuinely frozen
# simulation clock and should fail quickly. If this Python process temporarily
# receives no callbacks while another terminal still sees /clock advancing,
# allow a longer grace period because that is local callback/executor latency,
# not proof that simulation time stopped.
CLOCK_VALUE_STALL_TIMEOUT_WALL_SEC = 2.0
CLOCK_CALLBACK_GAP_TIMEOUT_WALL_SEC = 8.0
CLOCK_MESSAGE_FRESH_WALL_SEC = 0.5

# Test numbers that use the shared "settle -> per-repeat bag -> trial"
# mechanism (see MainWindow._run_repeated_trial). Tests 6 (UMBmark) and 9
# (Weight Transfer) are deliberately excluded: 6's "repeat" unit should be
# whole laps, not a single settled instant, and 9 already has its own
# forward/reverse pulse-cycle concept that isn't the same thing as a
# settle-then-trial repeat. Test 7 (Step Steer) needs a steady-state
# velocity baseline before its trial, not an at-rest settle, so it isn't
# included here either -- would need a different settle predicate first.
REPEAT_ENABLED_TEST_NUMS = {1, 2, 3, 4, 5, 8, 10}
CLOCK_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

try:
    from cobraflex_topics import CORE_TOPICS, SECONDARY_TOPICS
except ImportError:
    # Fallback so this script still runs standalone if cobraflex_topics.py
    # isn't sitting next to it. Strongly recommended to keep that file
    # present -- it's the single source of truth shared with the analyzer,
    # and is what caught the /odom-vs-/zed/zed_node/odom mismatch bug.
    CORE_TOPICS = [
    "/cmd_vel", "/zed/zed_node/odom", "/zed/zed_node/imu/data",
    "/cobraflex/wheel_speeds", "/cobraflex/battery", "/tf", "/tf_static",
    "/joint_states", "/cobraflex/feedback", "/zed/zed_node/pose",
    "/robot_description",
    ]

    SECONDARY_TOPICS = [
        "/clicked_point", 
        "/diagnostics", 
        "/move_base_simple/goal", 
        "/parameter_events", 
        "/rosout", 
        "/scan", 
        "/zed/joint_states", 
        "/zed/zed_description", 
        "/zed/zed_node/body_trk/skeletons", 
        "/zed/zed_node/depth/camera_info", 
        "/zed/zed_node/depth/depth_registered", 
        "/zed/zed_node/depth/depth_registered/camera_info", 
        "/zed/zed_node/depth/depth_registered/compressed", 
        "/zed/zed_node/depth/depth_registered/compressedDepth", 
        "/zed/zed_node/depth/depth_registered/theora", 
        "/zed/zed_node/left/color/rect/camera_info", 
        "/zed/zed_node/left/color/rect/image", 
        "/zed/zed_node/left/color/rect/image/camera_info", 
        "/zed/zed_node/left/color/rect/image/compressed", 
        "/zed/zed_node/left/color/rect/image/compressedDepth", 
        "/zed/zed_node/left/color/rect/image/theora", 
        "/zed/zed_node/obj_det/objects", 
        "/zed/zed_node/point_cloud/cloud_registered", 
        "/zed/zed_node/pose/status", 
        "/zed/zed_node/rgb/color/rect/camera_info", 
        "/zed/zed_node/rgb/color/rect/image", 
        "/zed/zed_node/rgb/color/rect/image/camera_info", 
        "/zed/zed_node/rgb/color/rect/image/compressed", 
        "/zed/zed_node/rgb/color/rect/image/compressedDepth", 
        "/zed/zed_node/rgb/color/rect/image/theora", 
        "/zed/zed_node/right/color/rect/camera_info", 
        "/zed/zed_node/right/color/rect/image", 
        "/zed/zed_node/right/color/rect/image/camera_info", 
        "/zed/zed_node/right/color/rect/image/compressed", 
        "/zed/zed_node/right/color/rect/image/compressedDepth", 
        "/zed/zed_node/right/color/rect/image/theora", 
        "/zed/zed_node/status/health", 
        "/zed/zed_node/status/heartbeat",
        "/cobraflex/contact_force/front_left",
        "/cobraflex/contact_force/rear_left",
        "/cobraflex/contact_force/front_right",
        "/cobraflex/contact_force/rear_right",
        "/cobraflex/wheel_cmd_debug",
        
        # Newly added topics synchronized from 'ros2 topic list' terminal output
        "/camera/camera_info",       # Camera calibration info for lane detection camera
        "/camera/image_raw_lane",    # Raw image data stream for lane detection
        "/clock",                    # ROS 2 simulation or rosbag time synchronization
        "/events/write_split",       # Internal rosbag2 topic for file splitting events
        "/odom_truth",               # Ground truth odometry data for accuracy evaluation
    ]

# ==========================================
# ROS 2 Core Control & Communication Node
# ==========================================
class VehicleControlNode(Node):
    def __init__(self):
        super().__init__('cobraflex_control_center')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)
        # Use ROS 2's standard Clock QoS so this subscriber remains
        # compatible with simulation clock publishers that use the
        # canonical best-effort/clock profile rather than default QoS.
        self.create_subscription(
            Clock, '/clock', self.clock_callback, CLOCK_QOS)
        
        # Fixed: Single subscription using Float32 to avoid ROS 2 type collision
        self.create_subscription(Float32, '/cobraflex/battery', self.battery_callback, 10)

        # Telemetry variables
        self.actual_v = 0.0
        self.actual_w = 0.0
        self.target_v = 0.0
        self.target_w = 0.0
        self.imu_ay = 0.0
        self.battery_info = "Unknown"
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_x = None
        self.last_y = None
        self.total_distance = 0.0
        
        self.traj_x = []
        self.traj_y = []
        self.history_v = []
        self.history_ay = []

        self.test_active = False
        self.test_status_text = "Idle"

        # Raw /clock observation is kept independent of the node's
        # use_sim_time parameter. The same controller therefore works on:
        #   - Isaac Sim: profile durations use an advancing /clock.
        #   - Real robot: no /clock is required; durations use monotonic time.
        # A topic that exists but is frozen at 0 is NOT considered usable.
        self.sim_clock_ns = None
        self.sim_clock_previous_ns = None
        self.sim_clock_advance_count = 0
        self.sim_clock_message_count = 0
        self.sim_clock_forward_count = 0
        self.sim_clock_reset_count = 0
        self.sim_clock_last_message_wall = None
        self.sim_clock_last_advance_wall = None
        self.sim_clock_lock = threading.Lock()
        
    def odom_callback(self, msg):
        self.actual_v = msg.twist.twist.linear.x
        self.actual_w = msg.twist.twist.angular.z
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        self.traj_x.append(self.current_x)
        self.traj_y.append(self.current_y)
        self.history_v.append(self.actual_v)
        
        if len(self.history_v) > 300: 
            self.history_v.pop(0)
        
        if self.last_x is not None and self.last_y is not None:
            self.total_distance += math.hypot(self.current_x - self.last_x, self.current_y - self.last_y)
            
        self.last_x = self.current_x
        self.last_y = self.current_y

    def imu_callback(self, msg):
        self.imu_ay = msg.linear_acceleration.y
        self.history_ay.append(self.imu_ay)
        if len(self.history_ay) > 300: 
            self.history_ay.pop(0)

    def battery_callback(self, msg):
        # Apply the hardware scaling correction when the reported value is amplified.
        # Values above 100 are treated as values scaled by a factor of 100.
        if msg.data > 100.0:
            real_voltage = msg.data / 100.0
        else:
            real_voltage = msg.data
            
        self.battery_info = f"{real_voltage:.2f} V"

    def clock_callback(self, msg):
        clock_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        received_wall = time.monotonic()
        with self.sim_clock_lock:
            previous = self.sim_clock_previous_ns
            self.sim_clock_message_count += 1
            self.sim_clock_last_message_wall = received_wall

            if previous is None:
                self.sim_clock_advance_count = 0
                self.sim_clock_last_advance_wall = None
            elif clock_ns < previous:
                # Stop/Reset makes simulation time move backwards. This is a
                # new epoch, not a stalled clock and not vehicle "no motion".
                self.sim_clock_reset_count += 1
                self.sim_clock_advance_count = 0
                self.sim_clock_last_advance_wall = None
            elif clock_ns > previous:
                self.sim_clock_advance_count += 1
                self.sim_clock_forward_count += 1
                self.sim_clock_last_advance_wall = received_wall

            self.sim_clock_previous_ns = clock_ns
            self.sim_clock_ns = clock_ns

    def sim_clock_snapshot(self):
        """Return one internally consistent /clock diagnostic snapshot."""
        with self.sim_clock_lock:
            return {
                "clock_ns": self.sim_clock_ns,
                "message_count": self.sim_clock_message_count,
                "forward_count": self.sim_clock_forward_count,
                "epoch_forward_count": self.sim_clock_advance_count,
                "reset_count": self.sim_clock_reset_count,
                "last_message_wall": self.sim_clock_last_message_wall,
                "last_advance_wall": self.sim_clock_last_advance_wall,
            }

    def sim_clock_is_advancing(self, max_age_sec=0.5):
        snapshot = self.sim_clock_snapshot()
        if (snapshot["clock_ns"] is None
                or snapshot["epoch_forward_count"] < 2
                or snapshot["last_advance_wall"] is None):
            return False
        return time.monotonic() - snapshot["last_advance_wall"] <= max_age_sec

    def sim_time_seconds(self):
        snapshot = self.sim_clock_snapshot()
        clock_ns = snapshot["clock_ns"]
        return None if clock_ns is None else clock_ns * 1e-9

    def reset_data(self):
        self.total_distance = 0.0
        self.traj_x.clear()
        self.traj_y.clear()
        self.history_v.clear()
        self.history_ay.clear()
        self.last_x = None
        self.last_y = None

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.target_v = float(v)
        self.target_w = float(w)
        self.cmd_pub.publish(msg)


# ==========================================
# PyQt5 GUI
# ==========================================
class MainWindow(QMainWindow):
    # Worker thread can't touch Qt widgets directly (PyQt is not thread-safe
    # for that). Emitting a signal from the worker thread auto-queues the
    # slot to run on the GUI thread instead -- the standard safe pattern.
    recording_check_ready = pyqtSignal(list)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.bag_process = None
        self.record_start_time = 0
        self.ui_lock = threading.Lock()
        self.profile_clock_mode = "steady"
        
        self.initUI()
        self.recording_check_ready.connect(self._show_recording_check_warning)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_dashboard)
        self.timer.start(100)

    def initUI(self):
        self.setWindowTitle('CobraFlex Kinematics and Dynamics Acquisition Console')
        self.resize(1400, 850)
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # ------------------------------------------
        # Left Panel: Configuration and Control
        # ------------------------------------------
        left_panel = QVBoxLayout()
        
        test_group = QGroupBox("Automated Profile Configuration")
        test_layout = QFormLayout()
        
        self.cb_test_type = QComboBox()
        self.cb_test_type.addItems([
            "1. Acceleration Test",
            "2. Full Braking Testing",
            "3. Steady-State Circular Driving",
            "4. In-place Skid-Steer Test",
            "5. Steady-State Max Velocity Test",
            "6. Square trajectory test /UMBmark",
            "7. Step Steer Input",
            "8. Coasting Testing",
            "9. Weight Transfer / Pitch Test",
            "10. Baseline Noise Floor Test",
        ])
        self.cb_test_type.currentIndexChanged.connect(self.update_input_fields)
        
        self.txt_v = QLineEdit("0.4")
        self.txt_w = QLineEdit("0.8")
        self.txt_duration = QLineEdit("8.0")
        self.txt_side = QLineEdit("2.0")
        self.txt_turn_factor = QLineEdit("1.0")
        self.txt_pulse_cycles = QLineEdit("3")
        self.txt_repeats = QLineEdit("1")
        self.txt_version = QLineEdit("1")
        self.cb_time_source = QComboBox()
        self.cb_time_source.addItem("Auto (advancing /clock else real time)", "auto")
        self.cb_time_source.addItem("Simulation (/clock required)", "sim")
        self.cb_time_source.addItem("Real robot (monotonic time)", "steady")
        
        test_layout.addRow("Test Profile:", self.cb_test_type)
        test_layout.addRow("Target Linear Vel (m/s):", self.txt_v)
        test_layout.addRow("Target Angular Vel (rad/s):", self.txt_w)
        test_layout.addRow("Profile Duration (s):", self.txt_duration)
        test_layout.addRow("Square Side Length (m):", self.txt_side)
        test_layout.addRow("Turn Factor (Test 6 only):", self.txt_turn_factor)
        test_layout.addRow("Pulse Cycles (Test 9 only):", self.txt_pulse_cycles)
        test_layout.addRow("Repeats (Tests 1,2,3,4,5,8,10):", self.txt_repeats)
        test_layout.addRow("Software Version:", self.txt_version)
        test_layout.addRow("Profile Timing Source:", self.cb_time_source)
        
        self.btn_run = QPushButton("Run Automated Test and Record")
        self.btn_run.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.btn_run.clicked.connect(self.execute_automated_sequence)
        
        self.btn_stop = QPushButton("Emergency Stop")
        self.btn_stop.setStyleSheet("background-color: #F44336; color: white; font-weight: bold; padding: 10px;")
        self.btn_stop.clicked.connect(self.emergency_stop)
        
        v_layout = QVBoxLayout()
        v_layout.addLayout(test_layout)
        v_layout.addWidget(self.btn_run)
        v_layout.addWidget(self.btn_stop)
        test_group.setLayout(v_layout)
        
        # ------------------------------------------
        # Rosbag Data Stream Full Selection
        # ------------------------------------------
        bag_group = QGroupBox("Telemetry Stream Logging Options")
        bag_layout = QVBoxLayout()
        
        self.list_topics = QListWidget()
        self.list_topics.setSelectionMode(QListWidget.MultiSelection)
        self.list_topics.setMaximumHeight(260)
        
        # 11 Core Topics (Checked by default).
        # Sourced from cobraflex_topics.py -- the names there were verified
        # 1:1 against the real `ros2 topic list` output on admit14-cobraflex.
        # (Previous version used bare names /odom, /imu/data, /wheel_speeds,
        # /battery which DON'T exist on this robot and recorded 0 messages.)
        core_topics = list(CORE_TOPICS)

        # Remaining secondary topics (unchecked by default, still selectable).
        secondary_topics = list(SECONDARY_TOPICS)
        
        # Add Core Topics
        for t in core_topics:
            item = QListWidgetItem(f"[Core] {t}")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked)
            self.list_topics.addItem(item)
            
        # Add Secondary Topics
        for t in secondary_topics:
            item = QListWidgetItem(t)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.list_topics.addItem(item)

        bag_layout.addWidget(self.list_topics)
        bag_group.setLayout(bag_layout)
        
        left_panel.addWidget(test_group)
        left_panel.addWidget(bag_group)
        
        # ------------------------------------------
        # Right Panel: Telemetry Dashboards
        # ------------------------------------------
        right_panel = QVBoxLayout()
        
        status_layout = QHBoxLayout()
        self.lbl_status = QLabel("Status: Idle")
        self.lbl_status.setFont(QFont("Arial", 14, QFont.Bold))
        self.lbl_time = QLabel("Bag Time: 00:00")
        self.lbl_time.setFont(QFont("Arial", 12))
        self.lbl_dist = QLabel("Distance: 0.000 m")
        self.lbl_dist.setFont(QFont("Arial", 12))
        self.lbl_battery = QLabel("Battery: Unknown")
        self.lbl_battery.setFont(QFont("Arial", 12))
        
        status_layout.addWidget(self.lbl_status)
        status_layout.addWidget(self.lbl_time)
        status_bar_space = QWidget()
        status_layout.addWidget(status_bar_space)
        status_layout.addWidget(self.lbl_dist)
        status_layout.addWidget(self.lbl_battery)
        
        self.figure = Figure(figsize=(8, 8))
        self.canvas = FigureCanvas(self.figure)
        
        self.ax_traj = self.figure.add_subplot(211)
        self.ax_vel = self.figure.add_subplot(212)
        
        right_panel.addLayout(status_layout)
        right_panel.addWidget(self.canvas)
        
        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 3)

        self.update_input_fields()

    def update_input_fields(self):
        idx = self.cb_test_type.currentIndex()
        self.txt_v.setEnabled(idx != 9)                      # idx9 = Baseline Noise Floor: no motion at all
        self.txt_w.setEnabled(idx in [2, 3, 5, 6])       
        self.txt_side.setEnabled(idx == 5)            
        self.txt_duration.setEnabled(idx != 5)
        self.txt_turn_factor.setEnabled(idx == 5)             # Test 6 (UMBmark) only
        self.txt_pulse_cycles.setEnabled(idx == 8)             # Test 9 (Weight Transfer) only
        self.txt_repeats.setEnabled((idx + 1) in REPEAT_ENABLED_TEST_NUMS)

    def _show_recording_check_warning(self, empty_topics):
        QMessageBox.warning(
            self, "Recording Data Check",
            "The following selected topics recorded zero messages:\n\n"
            + "\n".join(empty_topics) +
            "\n\nCommon causes:\n"
            "1) The topic name does not match `ros2 topic list`.\n"
            "2) The corresponding hardware node or driver is not running or is disconnected.\n"
            "3) QoS settings are incompatible.\n\n"
            "These streams are empty in this bag, so related KPIs may be unavailable or invalid."
        )

    def emergency_stop(self):
        # Only flip the flag + send one immediate stop command here.
        # Do NOT touch self.bag_process directly: testing_lifecycle_thread
        # is the single owner of that resource. If we also send SIGINT /
        # clear the reference from this (GUI) thread, we race with the
        # worker thread's own cleanup and can leave the bag process
        # un-waited-on (zombie / not flushed). _hold_command() in the
        # worker thread checks test_active every tick, so it will notice
        # this immediately and fall through to the normal post-buffer +
        # bag-stop cleanup on its own.
        self.ros_node.test_active = False
        self.ros_node.publish_cmd(0.0, 0.0)
        self.ros_node.test_status_text = "EMERGENCY STOPPED"

    def execute_automated_sequence(self):
        if self.ros_node.test_active:
            QMessageBox.warning(self, "Warning", "A test profile is currently active.")
            return
        
        test_string = self.cb_test_type.currentText()
        test_num_str = test_string.split(".")[0].strip()
        test_num = int(test_num_str)
        
        try:
            v = float(self.txt_v.text())
            w = float(self.txt_w.text())
            dur = float(self.txt_duration.text())
            side = float(self.txt_side.text())
            turn_factor = float(self.txt_turn_factor.text())
            pulse_cycles = float(self.txt_pulse_cycles.text())
            repeats = max(1, int(round(float(self.txt_repeats.text()))))
            version = self.txt_version.text().strip()
            time_source = self.cb_time_source.currentData()
        except ValueError:
            QMessageBox.critical(self, "Error", "Invalid numeric values detected in settings.")
            return

        v_str = str(v).replace('.', '')
        w_str = str(w).replace('.', '')
        side_str = str(side).replace('.', '')
        
        if test_num in [1, 2, 5, 8]:
            bag_name = f"test_0{test_num}_vel{v_str}_t{int(dur)}s_v{version}"
        elif test_num in [4]:
            bag_name = f"test_0{test_num}_w{w_str}_t{int(dur)}s_v{version}"
        elif test_num in [6]:
            bag_name = f"test_0{test_num}_vel{v_str}{w_str}_{side_str}m_v{version}"
        elif test_num == 9:
            bag_name = f"test_09_vel{v_str}_cyc{int(pulse_cycles)}_t{int(dur)}s_v{version}"
        elif test_num == 10:
            bag_name = f"test_10_static_t{int(dur)}s_v{version}"
        else:
            bag_name = f"test_0{test_num}_vel{v_str}{w_str}_t{int(dur)}s_v{version}"

        # Shared prefix only for every REPEAT_ENABLED_TEST_NUMS test -- each
        # repeat records its own bag named f"{bag_name}_rep{r+1}of{reps}"
        # (see MainWindow._run_repeated_trial), so no total-count suffix
        # belongs in bag_name itself.
        if test_num in REPEAT_ENABLED_TEST_NUMS:
            conflicts = [
                f"{bag_name}_rep{r+1}of{repeats}" for r in range(repeats)
                if os.path.exists(f"{bag_name}_rep{r+1}of{repeats}")
            ]
            if conflicts:
                QMessageBox.warning(
                    self, "Warning",
                    "The following per-repeat bag directories already exist. "
                    "Please increment version:\n\n" + "\n".join(conflicts))
                return
        elif os.path.exists(bag_name):
            QMessageBox.warning(self, "Warning", f"Directory '{bag_name}' already exists. Please increment version.")
            return

        # Extract topics dynamically, handling [Core] prefix perfectly
        selected_topics = []
        for index in range(self.list_topics.count()):
            item = self.list_topics.item(index)
            if item.checkState() == Qt.Checked:
                t_name = item.text()
                if t_name.startswith("[Core] "):
                    t_name = t_name.replace("[Core] ", "")
                selected_topics.append(t_name)

        if not selected_topics:
            QMessageBox.warning(self, "Warning", "Please select at least one topic to record!")
            return

        # Pass test_num (int) rather than the combo-box text. The old code
        # dispatched on test_type.startswith("1") etc., which silently
        # breaks the moment a two-digit test number shares a leading digit
        # with a single-digit one -- "10. Baseline..." also starts with "1",
        # so it would have wrongly run Test 1's acceleration profile.
        threading.Thread(target=self.testing_lifecycle_thread, 
                         args=(bag_name, selected_topics, test_num, v, w, dur, side,
                               turn_factor, pulse_cycles, repeats, time_source),
                         daemon=True).start()

    def _resolve_profile_clock(self, requested_mode, probe_sec=0.75):
        """Choose one clock for the entire test; never switch mid-profile."""
        if requested_mode == "steady":
            return "steady"

        probe_deadline = time.monotonic() + probe_sec
        while time.monotonic() < probe_deadline:
            if not self.ros_node.test_active:
                raise RuntimeError("Test cancelled while checking timing source")
            if self.ros_node.sim_clock_is_advancing():
                return "sim"
            time.sleep(0.02)

        if requested_mode == "sim":
            raise RuntimeError(
                "Simulation timing selected, but /clock did not advance. "
                "Press Play and verify that /clock changes before running the test."
            )
        return "steady"

    def _profile_now(self):
        if self.profile_clock_mode == "sim":
            value = self.ros_node.sim_time_seconds()
            if value is None:
                raise RuntimeError("/clock disappeared during a simulation-timed test")
            return value
        return time.monotonic()

    def _timed_loop(self, duration, tick, rate_hz=20.0, interruptible=True):
        """Run tick() for a duration measured by the selected profile clock."""
        if duration <= 0.0:
            return True
        period = 1.0 / max(rate_hz, 1.0)

        if self.profile_clock_mode == "sim":
            first_snapshot = self.ros_node.sim_clock_snapshot()
            if first_snapshot["clock_ns"] is None:
                raise RuntimeError(
                    "/clock disappeared during a simulation-timed test")
            last_clock_ns = first_snapshot["clock_ns"]
            last_reset_count = first_snapshot["reset_count"]
            elapsed = 0.0
            loop_start_wall = time.monotonic()
        else:
            start = time.monotonic()

        while True:
            if interruptible and not self.ros_node.test_active:
                return False

            if self.profile_clock_mode == "sim":
                snapshot = self.ros_node.sim_clock_snapshot()
                now_wall = time.monotonic()
                clock_ns = snapshot["clock_ns"]
                if clock_ns is None:
                    raise RuntimeError(
                        "/clock disappeared during a simulation-timed test")

                if snapshot["reset_count"] != last_reset_count:
                    # Do not add the backwards jump to elapsed time. Resume
                    # from the first value in the new clock epoch.
                    print(
                        "[Clock] simulation reset observed during timed phase; "
                        f"epoch {last_reset_count}->{snapshot['reset_count']}, "
                        f"new baseline={clock_ns * 1e-9:.9f}s"
                    )
                    last_reset_count = snapshot["reset_count"]
                    last_clock_ns = clock_ns
                elif clock_ns < last_clock_ns:
                    # Defensive fallback if a snapshot straddled a callback.
                    last_clock_ns = clock_ns
                elif clock_ns > last_clock_ns:
                    elapsed += (clock_ns - last_clock_ns) * 1e-9
                    last_clock_ns = clock_ns

                last_advance_wall = snapshot["last_advance_wall"]
                progress_age = (
                    now_wall - last_advance_wall
                    if last_advance_wall is not None
                    else now_wall - loop_start_wall
                )
                last_message_wall = snapshot["last_message_wall"]
                message_age = (
                    now_wall - last_message_wall
                    if last_message_wall is not None
                    else float("inf")
                )

                # Case 1: callbacks remain fresh but all carry an unchanged
                # value. This is a true frozen /clock value.
                if (progress_age > CLOCK_VALUE_STALL_TIMEOUT_WALL_SEC
                        and message_age <= CLOCK_MESSAGE_FRESH_WALL_SEC):
                    raise RuntimeError(
                        "/clock callbacks are arriving but the clock value "
                        f"has not advanced for {progress_age:.2f}s wall time "
                        f"(clock={clock_ns * 1e-9:.9f}s, "
                        f"messages={snapshot['message_count']}, "
                        f"forward={snapshot['forward_count']}, "
                        f"resets={snapshot['reset_count']}). "
                        "This is independent of whether the vehicle moves."
                    )

                # Case 2: this process receives no /clock callbacks. Give the
                # executor a longer grace period so brief GUI/CPU contention
                # is not mislabeled as a stopped Isaac Sim clock.
                if message_age > CLOCK_CALLBACK_GAP_TIMEOUT_WALL_SEC:
                    raise RuntimeError(
                        "this controller received no /clock callback for "
                        f"{message_age:.2f}s wall time "
                        f"(last clock={clock_ns * 1e-9:.9f}s, "
                        f"messages={snapshot['message_count']}, "
                        f"forward={snapshot['forward_count']}, "
                        f"resets={snapshot['reset_count']}). "
                        "If `ros2 topic echo /clock` still advances, the "
                        "problem is local callback/executor starvation, not "
                        "vehicle No Motion."
                    )

                if elapsed >= duration:
                    return True
            elif time.monotonic() - start >= duration:
                return True

            tick()
            time.sleep(period)

    def _wait_duration(self, duration, interruptible=True):
        """Wait without republishing a command, using the profile clock."""
        return self._timed_loop(
            duration, lambda: None, rate_hz=50.0,
            interruptible=interruptible)

    def _wait_for_settle(self, w_threshold=0.02, v_threshold=None,
                          confirm_sec=0.3, timeout_sec=5.0):
        """
        Block until the chassis's actual angular velocity (and, if
        `v_threshold` is given, linear velocity too) has stayed below
        threshold for `confirm_sec` continuous seconds, or `timeout_sec`
        is reached (whichever first). Ported from
        angular_velocity_sweep_v7.py's wait_for_settle().

        `v_threshold=None` (default) skips the linear-velocity check
        entirely, preserving the original Test-4-only behavior (pure
        in-place rotation, where only w mattered). Tests 1/2/3/5/8/10
        (added 2026-07-20, see _run_repeated_trial) command nonzero v and
        should pass an explicit v_threshold so a repeat doesn't start
        while the chassis still has residual translational motion from
        the previous repeat's coast-down.

        Why this matters: near the stick/slip friction transition, tiny
        differences in the chassis's starting state can flip which side
        of the transition a trial lands on, producing large point-to-point
        variance for the SAME commanded velocity. Publishes (0,0)
        throughout so the vehicle actively brakes to rest instead of just
        coasting.

        2026-07-20 fix: `confirm_sec`/`timeout_sec` are now measured on
        the profile clock (self._profile_now()) instead of wall-clock
        time.time(). Previously this function was the one piece of the
        script that bypassed profile_clock_mode entirely, so in a
        simulation-timed test with RTF != 1.0 (observed 0.84-0.85
        onsite), "timeout_sec=5.0" corresponded to a different number of
        *simulation* seconds depending on how loaded the machine was at
        that moment -- i.e. the settle duration wasn't actually
        controlled, only the wall-clock budget was. This makes settle
        consistent with _hold_command/_wait_duration, which already went
        through _profile_now() via _timed_loop().

        Returns (settled, elapsed_profile_sec):
          - settled=False if interrupted by Emergency Stop; otherwise True
            (including the timeout case, where it proceeds anyway -- same
            behavior as the sweep tool, just flagged in the status text).
          - elapsed_profile_sec is how long settle actually took, on the
            same clock as the rest of the profile (sim-seconds when
            profile_clock_mode=='sim'). This was previously not measured
            at all; callers should log it per repeat so the actual settle
            time used for each trial is on record instead of assumed.
        """
        below_since = None
        start = self._profile_now()
        deadline = start + timeout_sec
        while True:
            if not self.ros_node.test_active:
                return False, self._profile_now() - start
            self.ros_node.publish_cmd(0.0, 0.0)
            now = self._profile_now()
            w_ok = abs(self.ros_node.actual_w) < w_threshold
            v_ok = True if v_threshold is None else abs(self.ros_node.actual_v) < v_threshold
            if w_ok and v_ok:
                if below_since is None:
                    below_since = now
                elif now - below_since >= confirm_sec:
                    return True, now - start
            else:
                below_since = None
            if now >= deadline:
                self.ros_node.test_status_text = (
                    f"Settle timeout ({timeout_sec:.1f}s profile-clock) reached -- "
                    f"proceeding with residual v={self.ros_node.actual_v:.3f} m/s, "
                    f"w={self.ros_node.actual_w:.3f} rad/s"
                )
                return True, now - start
            # Poll interval itself stays on wall-clock -- only the measured
            # durations (confirm_sec/timeout_sec/elapsed) moved to the
            # profile clock, so this sleep does not need to change.
            time.sleep(0.02)

    def _hold_command(self, v, w, duration, rate_hz=20.0, interruptible=True):
        """Publish cmd_vel repeatedly for `duration` seconds instead of a
        single publish_cmd() + time.sleep().

        Why this matters for rosbag recording:
        - The old code called publish_cmd() exactly ONCE per phase, then
          blocked in time.sleep(). /cmd_vel uses default (volatile) QoS,
          so a single message published before the `ros2 bag record`
          subprocess has finished discovering/subscribing to the topic
          is silently dropped -- not queued, just lost. With only one
          message per phase, losing that one message means the bag has
          NO /cmd_vel data for that whole phase.
        - Many motor controllers also apply a cmd_vel watchdog timeout
          (e.g. zero the command if no new message arrives within
          ~0.5s). A single message per multi-second phase will look
          like the target velocity dropped to 0 shortly after each
          command, even though the GUI/script "intended" it to hold v.
        Publishing continuously at rate_hz fixes both: it gives the bag
        recorder many chances to catch a message even if it's not ready
        immediately, and it satisfies any downstream command watchdog.

        Returns False if interrupted early by Emergency Stop
        (test_active became False), True if it ran to completion.
        """
        return self._timed_loop(
            duration,
            lambda: self.ros_node.publish_cmd(v, w),
            rate_hz=rate_hz,
            interruptible=interruptible,
        )

    def _check_recorded_topics(self, bag_name, requested_topics):
        """After a recording finishes, parse the bag's metadata.yaml and
        return the subset of `requested_topics` that ended up with ZERO
        recorded messages.

        This exists specifically because this project already hit this
        failure mode once: Core Topics were typed as bare names that don't
        exist in `ros2 topic list`, so `ros2 bag record` quietly recorded
        nothing for them with no error at all. Checking automatically after
        every run means that's caught immediately instead of being
        discovered later while staring at an empty plot in the analyzer.
        """
        meta_path = os.path.join(bag_name, "metadata.yaml")
        if not os.path.exists(meta_path):
            return []
        try:
            import yaml
            with open(meta_path, "r") as f:
                meta = yaml.safe_load(f)
            counts = {}
            for entry in meta["rosbag2_bagfile_information"]["topics_with_message_count"]:
                counts[entry["topic_metadata"]["name"]] = entry["message_count"]
            return [t for t in requested_topics if counts.get(t, 0) == 0]
        except Exception as e:
            print(f"Recording sanity-check skipped ({e})")
            return []

    def _subscriber_counts(self, topics):
        """Return the ROS-graph subscriber count for each selected topic."""
        counts = {}
        for topic in topics:
            try:
                counts[topic] = int(self.ros_node.count_subscribers(topic))
            except Exception:
                counts[topic] = 0
        return counts

    def _advertised_topics(self, topics):
        """Selected topics that currently have at least one publisher.

        rosbag2 can only create a type-correct subscription after discovering
        a publisher.  Topics without a current publisher are left to the
        existing post-record 0-message check; they must not make recorder
        readiness impossible forever.
        """
        advertised = []
        for topic in topics:
            try:
                if self.ros_node.get_publishers_info_by_topic(topic):
                    advertised.append(topic)
            except Exception:
                pass
        return advertised

    def _wait_for_bag_recorder_ready(self, proc, topics, baseline_counts,
                                     timeout_wall_sec=RECORDER_READY_TIMEOUT_WALL_SEC):
        """Wait until the newly launched rosbag2 process has subscribed.

        Readiness is based on the ROS graph, not a blind sleep: every selected
        topic that had a publisher when recording started must gain at least
        one subscriber relative to its pre-launch baseline.  The condition
        must remain true briefly to avoid accepting a transient graph update.

        The timeout uses monotonic wall time because DDS discovery continues
        independently of simulation time.  Returns
        (elapsed_wall_sec, ready_topics, final_counts).
        """
        ready_topics = self._advertised_topics(topics)
        if not ready_topics:
            raise RuntimeError(
                "Recorder readiness cannot be verified: none of the selected "
                "topics currently has a publisher"
            )

        start = time.monotonic()
        deadline = start + timeout_wall_sec
        all_matched_since = None
        final_counts = dict(baseline_counts)

        while time.monotonic() < deadline:
            if not self.ros_node.test_active:
                raise RuntimeError("Test cancelled while waiting for rosbag recorder readiness")
            return_code = proc.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"ros2 bag record exited before becoming ready (code {return_code})"
                )

            final_counts = self._subscriber_counts(ready_topics)
            missing = [
                topic for topic in ready_topics
                if final_counts.get(topic, 0) <= baseline_counts.get(topic, 0)
            ]
            now = time.monotonic()
            if not missing:
                if all_matched_since is None:
                    all_matched_since = now
                elif now - all_matched_since >= RECORDER_READY_CONFIRM_WALL_SEC:
                    return now - start, ready_topics, final_counts
            else:
                all_matched_since = None
            time.sleep(RECORDER_GRAPH_POLL_WALL_SEC)

        final_counts = self._subscriber_counts(ready_topics)
        missing = [
            topic for topic in ready_topics
            if final_counts.get(topic, 0) <= baseline_counts.get(topic, 0)
        ]
        raise RuntimeError(
            "rosbag recorder readiness timeout after "
            f"{timeout_wall_sec:.1f}s; no new recorder subscription on: "
            + ", ".join(missing)
        )

    def _start_bag_recording(self, bag_name, topics):
        """Start `ros2 bag record -o bag_name topics...` and register the
        process under self.bag_process (used by update_dashboard's Rec
        Time display), then verify DDS/topic discovery from the ROS graph.

        A fixed sleep is not sufficient: onsite runs showed that one second
        sometimes elapsed before odometry/joint-state subscriptions were
        matched.  Motion is therefore forbidden until readiness succeeds.
        """
        baseline_counts = self._subscriber_counts(topics)
        cmd = ['ros2', 'bag', 'record', '-o', bag_name] + topics
        proc = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with self.ui_lock:
            self.bag_process = proc
        self.record_start_time = time.time()
        self.ros_node.test_status_text = (
            f"Waiting for rosbag recorder topic discovery: {bag_name}"
        )
        try:
            elapsed, ready_topics, final_counts = self._wait_for_bag_recorder_ready(
                proc, topics, baseline_counts)
        except Exception:
            # Flush/close the partial bag before propagating the failure.
            # The directory is intentionally retained as evidence and must
            # not be mistaken for a valid formal run.
            self._stop_bag_recording()
            raise

        count_summary = ", ".join(
            f"{topic}:{baseline_counts.get(topic, 0)}->{final_counts.get(topic, 0)}"
            for topic in ready_topics
        )
        print(
            f"[Recorder Ready] bag={bag_name} | elapsed_wall={elapsed:.3f}s | "
            f"subscriber_counts={count_summary}"
        )
        return proc

    def _stop_bag_recording(self, timeout_sec=10):
        """Stop whatever bag is currently recording (SIGINT, then wait for
        rosbag2 to flush; kill as a last resort). No-op if nothing is
        recording. Returns the process handle that was stopped, or None.
        """
        with self.ui_lock:
            proc = self.bag_process
            self.bag_process = None
        if proc is None:
            return None
        if proc.poll() is not None:
            proc.wait()
            return proc        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            self.ros_node.test_status_text = "Bag recorder unresponsive, forcing stop..."
            proc.kill()
            proc.wait()
        return proc

    def _run_repeated_trial(self, bag_name, topics, repeats, trial_label,
                             trial_fn, w_threshold=0.02, v_threshold=0.02):
        """Shared 'settle -> per-repeat bag -> trial' loop.

        2026-07-20: extracted from what used to be Test 4's own inline
        loop, so Tests 1/2/3/5/8/10 (which also want repeated trials from
        a controlled at-rest starting state) don't each need a hand-copied
        version of the same settle/bag/log bookkeeping.

        trial_fn: zero-arg callable that runs one repeat's command
        sequence (typically a closure wrapping one or more
        self._hold_command(...) calls) and returns True/False the same
        way _hold_command does (False = interrupted by Emergency Stop).

        v_threshold=0.02 (m/s) is a default, not a universal truth -- it
        was chosen as "same order of magnitude as the existing w
        threshold", not measured against actual sensor noise floor.
        Revisit if it turns out to be too tight/loose for a given test's
        settle time.
        """
        for r in range(repeats):
            if not self.ros_node.test_active: break
            self.ros_node.test_status_text = f"{trial_label} Repeat {r+1}/{repeats}: Confirming chassis at rest"
            settled, settle_elapsed = self._wait_for_settle(
                w_threshold=w_threshold, v_threshold=v_threshold)
            if not settled:
                break
            if not self.ros_node.test_active: break

            init_x = self.ros_node.current_x
            init_y = self.ros_node.current_y
            init_v = self.ros_node.actual_v
            init_w = self.ros_node.actual_w
            sim_t = self.ros_node.sim_time_seconds()
            print(
                f"[{trial_label} Repeat {r+1}/{repeats}] settle_elapsed="
                f"{settle_elapsed:.3f}s (profile clock, "
                f"mode={self.profile_clock_mode}) | init_pose="
                f"({init_x:.4f},{init_y:.4f}) | init_actual_v={init_v:.4f} "
                f"m/s | init_actual_w={init_w:.4f} rad/s | sim_t="
                f"{'N/A' if sim_t is None else f'{sim_t:.3f}s'}"
            )

            rep_bag_name = f"{bag_name}_rep{r+1}of{repeats}"
            self.ros_node.test_status_text = (
                f"{trial_label} Repeat {r+1}/{repeats}: Starting bag {rep_bag_name}"
            )
            self._start_bag_recording(rep_bag_name, topics)

            # Recorder readiness has already been verified above.  Now log a
            # complete 2.0 profile-clock seconds of zero command so the bag
            # contains the entire 0-2s post-command transient once motion
            # begins.  This is measured in simulation seconds when mode=sim.
            self.ros_node.test_status_text = (
                f"{trial_label} Repeat {r+1}/{repeats}: "
                f"{RECORDER_PREROLL_PROFILE_SEC:.1f}s zero-cmd pre-roll"
            )
            pre_roll_ok = self._hold_command(
                0.0, 0.0, RECORDER_PREROLL_PROFILE_SEC)
            if not pre_roll_ok:
                self._stop_bag_recording()
                break

            print(
                f"[{trial_label} Repeat {r+1}/{repeats}] recorder_ready=yes | "
                f"zero_cmd_preroll={RECORDER_PREROLL_PROFILE_SEC:.3f}s "
                f"(profile clock, mode={self.profile_clock_mode})"
            )

            self.ros_node.test_status_text = f"{trial_label} Repeat {r+1}/{repeats}: Executing profile"
            trial_ok = trial_fn()

            self._stop_bag_recording()
            empty = self._check_recorded_topics(rep_bag_name, topics)
            if empty:
                print(
                    f"[{trial_label} Repeat {r+1}/{repeats}] WARNING: "
                    f"{rep_bag_name} has 0 messages on: {empty}"
                )
                self.recording_check_ready.emit(empty)

            if not trial_ok: break

    def testing_lifecycle_thread(self, bag_name, topics, test_num, v, w,
                                 duration, side_length, turn_factor,
                                 pulse_cycles, repeats, requested_time_source):
        self.ros_node.test_active = True
        self.ros_node.reset_data()
        self.btn_run.setEnabled(False)
        lifecycle_error = None

        try:
            # Resolve Auto/Simulation/Real BEFORE starting rosbag so the
            # topic set matches the environment. /odom_truth and /clock are
            # sim-only here: always record both for a simulation-timed run,
            # and remove them for a real/steady-timed run to avoid guaranteed
            # 0-message warnings on the physical robot.
            self.profile_clock_mode = self._resolve_profile_clock(requested_time_source)
            topics = list(topics)
            if self.profile_clock_mode == "sim":
                for sim_topic in (SIM_ODOM_TRUTH_TOPIC, SIM_CLOCK_TOPIC):
                    if sim_topic not in topics:
                        topics.append(sim_topic)
            else:
                topics = [
                    t for t in topics
                    if t not in (SIM_ODOM_TRUTH_TOPIC, SIM_CLOCK_TOPIC)
                ]

            if test_num in REPEAT_ENABLED_TEST_NUMS:
                # These tests record one bag PER REPEAT, started only
                # after settle completes (see _run_repeated_trial) -- the
                # #112 protocol is "reset -> settle -> THEN record", so
                # the settle phase itself is deliberately left out of the
                # bag instead of being captured at the front of one long
                # multi-repeat recording.
                self.ros_node.test_status_text = (
                    "Per-repeat bag recording (starts after each settle)"
                )
            else:
                self.ros_node.test_status_text = "Initializing Rosbag recorder..."
                self._start_bag_recording(bag_name, topics)

            clock_label = ("ROS /clock (simulation time)" if self.profile_clock_mode == "sim"
                           else "monotonic time (real robot)")
            self.ros_node.test_status_text = f"Timing locked: {clock_label}; pre-buffer 2s"
            self._hold_command(0.0, 0.0, 2.0)

            if test_num == 1: 
                self._run_repeated_trial(
                    bag_name, topics, repeats, "Test1 Acceleration",
                    lambda: self._hold_command(v, 0.0, duration))

            elif test_num == 2: 
                def _trial_test2():
                    if self._hold_command(v, 0.0, duration):
                        self.ros_node.test_status_text = "Injecting Hard Brake Command"
                        return self._hold_command(0.0, 0.0, 3.0)
                    return False
                self._run_repeated_trial(
                    bag_name, topics, repeats, "Test2 Full Braking", _trial_test2)

            elif test_num == 3: 
                self._run_repeated_trial(
                    bag_name, topics, repeats, "Test3 Circular Driving",
                    lambda: self._hold_command(v, w, duration))

            elif test_num == 4: 
                # 2026-07-20 (#112 protocol fix): each repeat gets its own
                # bag (test_0..._rep{N}of{M}), opened only after settle
                # completes, plus a console line recording the settle
                # duration (profile-clock seconds) and the initial pose --
                # previously neither was recorded anywhere, so a repeat's
                # starting condition had to be guessed post-hoc from
                # /cmd_vel edges in one shared bag. Now shares
                # _run_repeated_trial with Tests 1/2/3/5/8/10 instead of
                # its own hand-copied loop.
                self._run_repeated_trial(
                    bag_name, topics, repeats, "Test4 In-place Skid-Steer",
                    lambda: self._hold_command(0.0, w, duration))

            elif test_num == 5: 
                self._run_repeated_trial(
                    bag_name, topics, repeats, "Test5 Max Velocity",
                    lambda: self._hold_command(v, 0.0, duration))

            elif test_num == 6: 
                t_straight = side_length / v if v != 0 else 0
                t_turn = ((math.pi / 2.0) / w) * turn_factor if w != 0 else 0
                
                for edge in range(4):
                    if not self.ros_node.test_active: break
                    self.ros_node.test_status_text = f"UMBmark Edge {edge+1}/4: Straight"
                    if not self._hold_command(v, 0.0, t_straight): break
                    
                    if edge < 3:
                        if not self.ros_node.test_active: break
                        self.ros_node.test_status_text = f"UMBmark Edge {edge+1}/4: Turning"
                        if not self._hold_command(0.0, w, t_turn): break

            elif test_num == 7: 
                self.ros_node.test_status_text = "Establishing straight trajectory base"
                if self._hold_command(v, 0.0, 3.0):
                    self.ros_node.test_status_text = "Injecting Step Steering Step"
                    self._hold_command(v, w, duration)

            elif test_num == 8: 
                def _trial_test8():
                    if self._hold_command(v, 0.0, duration):
                        self.ros_node.test_status_text = "Cutting actuator power (Coasting)"
                        # Actually publish the stop command (was: a bare
                        # attribute assignment `self.ros_node.target_v = 0.0`
                        # that never touched /cmd_vel at all -- the bag kept
                        # recording the last nonzero command and the
                        # actuator was never cut).
                        self.ros_node.publish_cmd(0.0, 0.0)
                        return self._wait_duration(6.0)
                    return False
                self._run_repeated_trial(
                    bag_name, topics, repeats, "Test8 Coasting", _trial_test8)

            elif test_num == 9:
                # Weight Transfer / Pitch Test -- from the project's own
                # Sim2Real spreadsheet (Overall sheet, row "8 Weight
                # Transfer/Pitch Test"), targeting Isaac Sim's centerOfMass
                # calibration. Not in the original 8-test menu.
                # Protocol: alternate sharp forward/reverse pulses to excite
                # pitch (dive/squat) oscillation; the analyzer reads the
                # peak pitch rate (IMU wy) vs the achieved Ax swing.
                # Uses the dedicated Pulse Cycles field, not Repeats --
                # this test's own forward/reverse pulse concept isn't the
                # same thing as a settle-then-trial repeat.
                cycles = max(1, int(round(pulse_cycles)))
                for c in range(cycles):
                    if not self.ros_node.test_active: break
                    self.ros_node.test_status_text = f"Weight-Transfer Pulse {c+1}/{cycles}: Forward"
                    if not self._hold_command(v, 0.0, duration): break
                    self.ros_node.test_status_text = f"Weight-Transfer Pulse {c+1}/{cycles}: Reverse"
                    if not self._hold_command(-v, 0.0, duration): break

            elif test_num == 10:
                # Baseline Noise Floor Test (new -- not in the spreadsheet,
                # recommended addition). Vehicle stays fully stationary the
                # whole time: zero velocity commanded throughout. Lets the
                # analyzer report IMU static bias/noise and ZED VIO drift
                # while at rest, which every other test's KPI should really
                # be interpreted relative to. Settle is trivial here since
                # the target is already (0,0), but still goes through
                # _run_repeated_trial for a consistent per-repeat bag +
                # log record across repeats.
                self._run_repeated_trial(
                    bag_name, topics, repeats, "Test10 Baseline Noise Floor",
                    lambda: self._hold_command(0.0, 0.0, duration))

        except Exception as thread_err:
            lifecycle_error = str(thread_err)
            print(f"Lifecycle Execution Error: {thread_err}")
            self.ros_node.publish_cmd(0.0, 0.0)
            # Cleanup must not depend on a clock that may have caused the
            # failure (for example /clock frozen at zero).
            self.profile_clock_mode = "steady"
            
        self.ros_node.test_status_text = "Post-buffer logging active (1s)..."
        # interruptible=False: this final stop command must always be sent
        # and recorded, even if Emergency Stop already flipped test_active.
        try:
            self._hold_command(0.0, 0.0, 1.0, interruptible=False)
        except Exception as cleanup_err:
            if lifecycle_error is None:
                lifecycle_error = str(cleanup_err)
            self.profile_clock_mode = "steady"
            self.ros_node.publish_cmd(0.0, 0.0)
        
        if self.bag_process is not None:
            self.ros_node.test_status_text = "Finalizing and flushing data cache..."
            self._stop_bag_recording()

        # For Test 4, per-repeat bags were already checked (and any empty
        # ones warned about) inside the loop above; bag_name itself was
        # never created as a directory, so this correctly reports no
        # empty topics rather than a false "bag_name not found" issue.
        empty_topics = self._check_recorded_topics(bag_name, topics)
        if lifecycle_error is not None:
            self.ros_node.test_active = False
            self.ros_node.test_status_text = (
                f"Sequence FAILED ({bag_name}): {lifecycle_error}"
            )
        elif empty_topics:
            self.recording_check_ready.emit(empty_topics)
            self.ros_node.test_active = False
            self.ros_node.test_status_text = (
                f"Sequence Complete ({bag_name}) -- WARNING: "
                f"{len(empty_topics)} topic(s) recorded 0 messages!"
            )
        else:
            self.ros_node.test_active = False
            self.ros_node.test_status_text = f"Sequence Complete ({bag_name})"
        self.btn_run.setEnabled(True)

    def update_dashboard(self):
        self.lbl_status.setText(f"Status: {self.ros_node.test_status_text}")
        self.lbl_dist.setText(f"Distance: {self.ros_node.total_distance:.2f} m")
        self.lbl_battery.setText(f"Battery: {self.ros_node.battery_info}")
        
        with self.ui_lock:
            if self.bag_process is not None:
                mins, secs = divmod(int(time.time() - self.record_start_time), 60)
                self.lbl_time.setText(f"Rec Time: {mins:02d}:{secs:02d}")
                self.lbl_time.setStyleSheet("color: red; font-weight: bold;")
            else:
                self.lbl_time.setText("Rec Time: 00:00")
                self.lbl_time.setStyleSheet("color: black;")

        self.ax_traj.clear()
        self.ax_vel.clear()
        
        if len(self.ros_node.traj_x) > 0:
            self.ax_traj.plot(self.ros_node.traj_x, self.ros_node.traj_y, 'b-', label='ZED Odometry')
            self.ax_traj.plot(self.ros_node.traj_x[0], self.ros_node.traj_y[0], 'go') 
            self.ax_traj.plot(self.ros_node.traj_x[-1], self.ros_node.traj_y[-1], 'ro') 
        self.ax_traj.set_title("Real-Time 2D Trajectory Map")
        self.ax_traj.axis('equal')
        self.ax_traj.grid(True, linestyle='--')
        
        if len(self.ros_node.history_v) > 0:
            self.ax_vel.plot(self.ros_node.history_v, 'orange', label='Velocity (m/s)', linewidth=2)
            self.ax_vel.plot(self.ros_node.history_ay, 'red', alpha=0.4, label='Lateral Accel (m/s^2)', linewidth=1)
        self.ax_vel.set_title("Chassis Dynamic Response Indicators")
        self.ax_vel.grid(True, linestyle='--')
        self.ax_vel.legend(loc='upper right')
        
        self.canvas.draw()


def main(args=None):
    rclpy.init(args=args)
    ros_node = VehicleControlNode()
    
    executor_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    executor_thread.start()
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = MainWindow(ros_node)
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()