import sys
import rclpy
import csv
import os
import threading
import socket
import cv2
import numpy as np
import time
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Int32
from PyQt6.QtWidgets import (QApplication, QWidget, QGridLayout, QPushButton, 
                             QLabel, QComboBox, QGroupBox, QHBoxLayout, QVBoxLayout, QLineEdit)
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap, QFont

from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

class Communicate(QObject):
    update_ui = pyqtSignal(list)
    update_cam = pyqtSignal(QImage)
    update_pose = pyqtSignal(list)

class JetCobotMasterDashboard(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'pc_dashboard_node')
        QWidget.__init__(self)
        
        self.joint_names = [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4', 
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ]

        self.actual_angles_backup = [0.0] * 6 
        self.config_file = os.path.join(os.getcwd(), "positions.csv")
        self.capture_dir = os.path.join(os.getcwd(), "captures")
        if not os.path.exists(self.capture_dir): os.makedirs(self.capture_dir)

        self.UDP_PORT = 9505
        self.current_angles = [0.0] * 6  
        self.current_pose = [0.0] * 6 
        self.latest_frame = None  
        self.memory = {i: [0.0]*6 for i in range(1, 6)}
        self.pose_memory = {i: [0.0]*6 for i in range(1, 6)} # 좌표 메모리 추가
        self.is_streaming = False
        
        # Robot connection status monitoring
        self.robot_connected = False
        self.last_encoder_time = time.time()
        self.connection_timeout = 1.0  # 1 second timeout
        self.connection_check_timer = self.create_timer(0.5, self.check_robot_connection)

        self.pub_angles = self.create_publisher(Float64MultiArray, 'target_angles', 10)
        self.pub_servo = self.create_publisher(Bool, 'servo_status', 10)
        self.pub_gripper = self.create_publisher(Int32, 'gripper_control', 10)
        self.pub_pose_target = self.create_publisher(Pose, 'goal_pose', 10)
        
        self.sub_current = self.create_subscription(Float64MultiArray, 'current_angles', self.receive_callback, 10)
        self.sub_pose = self.create_subscription(Pose, 'current_pose', self.pose_callback, 10)
        
        self.comm = Communicate()
        self.comm.update_ui.connect(self.update_display)
        self.comm.update_cam.connect(self.set_image)
        self.comm.update_pose.connect(self.update_pose_display)
        
        self.init_ui()
        self.auto_load_at_startup()

    def init_ui(self):
        self.setWindowTitle('JetCobot Master - Hybrid Control Full Dashboard')
        self.setMinimumWidth(1600)
        self.setMinimumHeight(900)
        
        # Apply unified stylesheet
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #1e1e1e; color: #ffffff; }
            QGroupBox { border: 1px solid #444; border-radius: 5px; margin-top: 10px; padding-top: 10px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }
            QPushButton { background-color: #0d47a1; color: white; border: none; border-radius: 4px; padding: 6px 12px; font-weight: bold; }
            QPushButton:hover { background-color: #1565c0; }
            QPushButton:pressed { background-color: #0a3d91; }
            QLineEdit, QComboBox { background-color: #2b2b2b; color: #ffffff; border: 1px solid #444; border-radius: 3px; padding: 4px; }
            QLabel { color: #ffffff; }
        """)
        master_layout = QHBoxLayout()

        self.main_font = QFont("Arial", 11, QFont.Weight.Bold)
        header_font = QFont("Arial", 10, QFont.Weight.Bold)

        # --- [좌측] 카메라 영역 ---
        vision_group = QGroupBox("📷 Camera Vision")
        vision_group.setFixedWidth(680)
        vision_layout = QVBoxLayout()
        vision_layout.setSpacing(10)
        self.cam_view = QLabel("UDP Video Waiting...")
        self.cam_view.setFixedSize(640, 480)
        self.cam_view.setStyleSheet("background-color: black; border: 2px solid #555; border-radius: 4px;")
        self.cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        cam_ctrl_layout = QHBoxLayout()
        cam_ctrl_layout.setSpacing(8)
        btn_cam_connect = QPushButton("🔌 CONNECT")
        btn_cam_disconnect = QPushButton("❌ DISCONNECT")
        btn_cam_capture = QPushButton("📸 CAPTURE")
        btn_cam_connect.clicked.connect(self.start_camera)
        btn_cam_disconnect.clicked.connect(self.stop_camera)
        btn_cam_capture.clicked.connect(self.capture_image)

        cam_ctrl_layout.addWidget(btn_cam_connect)
        cam_ctrl_layout.addWidget(btn_cam_disconnect)
        cam_ctrl_layout.addWidget(btn_cam_capture)
        vision_layout.addWidget(self.cam_view)
        vision_layout.addLayout(cam_ctrl_layout)
        vision_group.setLayout(vision_layout)

        # --- 통신 상태 모니터링 ---
        comm_group = QGroupBox("📡 Communication Status")
        comm_layout = QHBoxLayout()
        comm_layout.setSpacing(15)
        
        # Robot 연결 상태
        self.lbl_robot_status = QLabel("🔴 Robot: Disconnected")
        self.lbl_robot_status.setStyleSheet("color: #FF6B6B; font-weight: bold; padding: 8px; background-color: #1e1e1e; border-radius: 3px;")
        self.lbl_robot_status.setMinimumWidth(180)
        
        # Camera 연결 상태
        self.lbl_camera_status = QLabel("🔴 Camera: Disconnected")
        self.lbl_camera_status.setStyleSheet("color: #FF6B6B; font-weight: bold; padding: 8px; background-color: #1e1e1e; border-radius: 3px;")
        self.lbl_camera_status.setMinimumWidth(180)
        
        comm_layout.addWidget(self.lbl_robot_status)
        comm_layout.addWidget(self.lbl_camera_status)
        comm_layout.addStretch()
        comm_group.setLayout(comm_layout)
        comm_group.setMinimumHeight(90)
        comm_group.setMaximumHeight(90)

        # --- [우측] 제어 영역 ---
        robot_layout = QVBoxLayout()
        robot_layout.setSpacing(10)

        # 1. System Control
        sys_group = QGroupBox("⚙️ System Control")
        sys_group.setMinimumHeight(90)
        sys_group.setMaximumHeight(90)
        sys_h_layout = QHBoxLayout()
        sys_h_layout.setSpacing(8)
        for text, func in [("✓ Servo ON", lambda: self.send_servo(True)), ("✗ Servo OFF", lambda: self.send_servo(False)), 
                           ("🏠 HOME", self.go_home), ("✊ GRIP", lambda: self.control_gripper(1)), ("🖐️ UNGRIP", lambda: self.control_gripper(0))]:
            btn = QPushButton(text)
            btn.setFixedSize(105, 40)
            btn.clicked.connect(func)
            sys_h_layout.addWidget(btn)
        
        sys_h_layout.addStretch()
        sys_h_layout.addWidget(QLabel("Speed: 50"))
        sys_group.setLayout(sys_h_layout)

        # 2. Joint Jog Monitoring (저장/이동 복구)
        jog_group = QGroupBox("Joint Jog (Direct Angle Control)")
        jog_group.setMinimumHeight(380)
        grid = QGridLayout(); grid.setSpacing(8)
        headers = ["Axis", "Jog (+/-)", "Target", "Actual", "Error/F", "Pos 1", "Pos 2", "Pos 3", "Pos 4", "Pos 5"]
        for col, text in enumerate(headers): grid.addWidget(QLabel(text), 0, col, Qt.AlignmentFlag.AlignCenter)

        self.target_labels, self.enc_labels, self.error_labels, self.factor_inputs = [None]*6, [None]*6, [None]*6, [None]*6
        self.jog_step_inputs = [None]*6
        self.mem_labels = {i: [None]*6 for i in range(1, 6)}

        for i in range(6):
            row = 6 - i  # Reverse order: J6 at row 1, J1 at row 6
            lbl = QLabel(f"J{i+1}")
            lbl.setFixedWidth(80)
            lbl.setFont(self.main_font)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(lbl, row, 0)
            
            jog_box = QHBoxLayout(); btn_m, btn_p = QPushButton("-"), QPushButton("+")
            btn_m.setFixedSize(32, 32); btn_p.setFixedSize(32, 32)
            btn_m.clicked.connect(lambda ch, idx=i: self.jog(idx, -1))
            btn_p.clicked.connect(lambda ch, idx=i: self.jog(idx, 1))
            step_input = QLineEdit("1.0"); step_input.setFixedSize(40, 32); step_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.jog_step_inputs[i] = step_input
            jog_box.addWidget(btn_m); jog_box.addWidget(step_input); jog_box.addWidget(btn_p); grid.addLayout(jog_box, row, 1)
            
            self.target_labels[i] = self.create_label("0.0", "#2196F3", 55); grid.addWidget(self.target_labels[i], row, 2)
            self.enc_labels[i] = self.create_label("0.0", "#757575", 55); grid.addWidget(self.enc_labels[i], row, 3)
            
            err_f_box = QHBoxLayout(); err_lbl = self.create_label("0.0", "#f44336", 40)
            self.error_labels[i] = err_lbl
            f_input = QLineEdit("1.0"); f_input.setFixedSize(35, 32); f_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.factor_inputs[i] = f_input
            err_f_box.addWidget(err_lbl); err_f_box.addWidget(f_input); grid.addLayout(err_f_box, row, 4)

            for m_idx in range(1, 6):
                self.mem_labels[m_idx][i] = self.create_label("---", "#555", 55)
                grid.addWidget(self.mem_labels[m_idx][i], row, m_idx + 4)

        # Joint 저장/이동 버튼 복구
        for m_idx in range(1, 6):
            btn_vbox = QVBoxLayout(); btn_vbox.setSpacing(8)
            s_btn, m_btn = QPushButton("저장"), QPushButton("이동")
            s_btn.setFixedSize(60, 30); m_btn.setFixedSize(60, 30)
            s_btn.setStyleSheet("font-size: 10px; background-color: #455A64;")
            m_btn.setStyleSheet("font-size: 10px; background-color: #1E88E5;")
            s_btn.clicked.connect(lambda ch, m=m_idx: self.save_memory(m))
            m_btn.clicked.connect(lambda ch, m=m_idx: self.move_memory(m))
            btn_vbox.addWidget(s_btn); btn_vbox.addWidget(m_btn); grid.addLayout(btn_vbox, 7, m_idx + 4)

        jog_group.setLayout(grid)

        # 3. Cartesian Monitoring (이동 버튼 복구 및 통합 버튼)
        cart_group = QGroupBox("🎯 MoveIt Cartesian Precision Control")
        cart_group.setMinimumHeight(380)
        c_grid = QGridLayout()
        c_grid.setSpacing(12); c_grid.setRowMinimumHeight(7, 35)
        
        c_headers = ["Axis", "Jog", "Current", "Target Input", "", "P1", "P2", "P3", "P4", "P5"]
        for col, text in enumerate(c_headers):
            if text:  # Skip empty spacer
                label = QLabel(text)
                label.setStyleSheet("font-weight: bold;")
                c_grid.addWidget(label, 0, col, Qt.AlignmentFlag.AlignCenter)

        self.pose_labels, self.pose_inputs, self.pose_delta_inputs = [], [], []
        self.pose_mem_labels = {m: [None]*6 for m in range(1, 6)}
        axes = ["X(mm)", "Y(mm)", "Z(mm)", "R(°)", "P(°)", "Y(°)"]

        for i in range(6):
            row = i + 1
            axis_lbl = QLabel(axes[i])
            axis_lbl.setFixedWidth(80)
            axis_lbl.setFont(self.main_font)
            axis_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            c_grid.addWidget(axis_lbl, row, 0)

            # Jog controls: [-] [delta edit] [+] (placed before Current)
            jog_layout = QHBoxLayout()
            jog_layout.setSpacing(4)
            jbtn_m = QPushButton("-")
            jbtn_m.setFixedSize(32, 32)
            jbtn_p = QPushButton("+")
            jbtn_p.setFixedSize(32, 32)
            delta_input = QLineEdit("5.0")
            delta_input.setFixedSize(50, 28)
            delta_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.pose_delta_inputs.append(delta_input)
            jbtn_m.clicked.connect(lambda ch, idx=i: self.cart_jog(idx, -1))
            jbtn_p.clicked.connect(lambda ch, idx=i: self.cart_jog(idx, 1))
            jog_layout.addWidget(jbtn_m)
            jog_layout.addWidget(delta_input)
            jog_layout.addWidget(jbtn_p)
            c_grid.addLayout(jog_layout, row, 1)

            self.pose_labels.append(self.create_label("0.0", "#2e7d32", 65))
            c_grid.addWidget(self.pose_labels[i], row, 2)

            p_in = QLineEdit("0.0")
            p_in.setFixedSize(75, 32)
            p_in.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.pose_inputs.append(p_in)
            c_grid.addWidget(p_in, row, 3)

            for m_idx in range(1, 6):
                self.pose_mem_labels[m_idx][i] = self.create_label("---", "#555", 60)
                c_grid.addWidget(self.pose_mem_labels[m_idx][i], row, m_idx + 4)

        # Cartesian 이동 버튼 복구
        for m_idx in range(1, 6):
            pm_btn = QPushButton("이동")
            pm_btn.setFixedSize(60, 30)
            pm_btn.setStyleSheet("font-size: 11px; background-color: #2E7D32;")
            pm_btn.clicked.connect(lambda ch, m=m_idx: self.move_pose_memory(m))
            # Place per-memory move buttons in the same row (row 7) across P1..P5 columns
            c_grid.addWidget(pm_btn, 7, m_idx + 4, Qt.AlignmentFlag.AlignCenter)

        # [통합 이동 버튼] - place under Target Input column (adjusted index)
        self.btn_move_all = QPushButton("MOVE TO TARGET")
        self.btn_move_all.setFixedSize(160, 40)
        self.btn_move_all.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; border-radius: 5px;")
        self.btn_move_all.clicked.connect(self.move_to_input_pose)
        # place below the inputs (row 7, column 3)
        c_grid.addWidget(self.btn_move_all, 7, 3, 1, 1, Qt.AlignmentFlag.AlignCenter)

        cart_group.setLayout(c_grid)

        # 4. Automation & Status
        auto_group = QGroupBox("Automation Process")
        auto_group.setMaximumHeight(120)
        auto_layout = QHBoxLayout()
        self.btn_auto = QPushButton("START SEQUENCE"); self.btn_auto.setMinimumHeight(45)
        self.btn_auto.clicked.connect(self.start_auto_process)
        self.lbl_status = QLabel("Status: Idle"); self.lbl_status.setStyleSheet("color: #1565C0; font-weight: bold;")
        
        auto_layout.addWidget(self.btn_auto)
        auto_layout.addWidget(self.lbl_status)
        auto_group.setLayout(auto_layout)

        robot_layout.addWidget(sys_group)
        robot_layout.addWidget(jog_group)
        robot_layout.addWidget(cart_group)
        robot_layout.addStretch()
        
        # Left side layout: Communication + Automation + Camera
        left_layout = QVBoxLayout()
        left_layout.addWidget(comm_group)
        left_layout.addWidget(auto_group)
        left_layout.addWidget(vision_group)
        left_layout.addStretch()
        
        master_layout.addLayout(left_layout)
        master_layout.addLayout(robot_layout)
        self.setLayout(master_layout)
        self.show()

    def create_label(self, text, color, width=60):
        lbl = QLabel(text)
        lbl.setFixedSize(width, 32)
        lbl.setFont(self.main_font)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f"border: 1px solid {color}; background-color: white; color: black; border-radius: 3px;")
        return lbl

    # --- 콜백 및 제어 로직 ---
    def receive_callback(self, msg):
        # Validate the incoming Float64MultiArray message from arm_driver
        if not msg.data or len(msg.data) != 6:
            self.get_logger().warn("Invalid encoder message received: Expected 6 values.")
            return

        try:
            # The arm_driver publishes angles directly in degrees
            angles = list(msg.data)
            
            # Check if the angles are valid before updating
            if any(a is None or (isinstance(a, float) and (math.isnan(a) or math.isinf(a))) for a in angles):
                self.get_logger().error("Received invalid encoder values (NaN or Inf). Skipping update.")
                return
            
            # Update last encoder receive time (for connection monitoring)
            self.last_encoder_time = time.time()
            
            # Update backup and current angles
            self.actual_angles_backup = list(angles)
            if all(a == 0.0 for a in self.current_angles):
                self.current_angles = list(angles)
            
            # Emit signal to update UI
            self.comm.update_ui.emit(angles)
            
        except Exception as e:
            self.get_logger().error(f"Error processing encoder message: {e}")

    def pose_callback(self, msg):
        x, y, z = msg.position.x * 1000, msg.position.y * 1000, msg.position.z * 1000
        r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        self.comm.update_pose.emit([x, y, z, roll, pitch, yaw])

    def update_display(self, angles):
        if not angles or len(angles) != 6:
            self.get_logger().warn("Invalid angles data for UI update.")
            return

        for i in range(6):
            self.enc_labels[i].setText(f"{angles[i]:.1f}")
            self.target_labels[i].setText(f"{self.current_angles[i]:.1f}")
            err = self.current_angles[i] - angles[i]
            self.error_labels[i].setText(f"{err:.1f}")
            self.error_labels[i].setStyleSheet(
                f"background-color: {'#C8E6C9' if abs(err) < 0.5 else '#FFCDD2'}; color: black;"
            )

    def update_pose_display(self, pose_vals):
        self.current_pose = pose_vals
        for i in range(6): 
            self.pose_labels[i].setText(f"{pose_vals[i]:.1f}")
            if self.pose_inputs[i].text() == "0.0": self.pose_inputs[i].setText(f"{pose_vals[i]:.1f}")

    def jog(self, idx, direction):
        try:
            step = float(self.jog_step_inputs[idx].text())
            self.current_angles[idx] += (direction * step)
            self.publish_angles()
        except: pass

    def cart_jog(self, idx, direction):
        """Adjust the Cartesian target input by delta (mm) using the jog +/- buttons."""
        try:
            delta = float(self.pose_delta_inputs[idx].text())
        except Exception:
            return
        try:
            # prefer current displayed pose value
            current = float(self.pose_labels[idx].text())
        except Exception:
            try:
                current = float(self.pose_inputs[idx].text())
            except Exception:
                current = 0.0

        new_val = current + (direction * delta)
        self.pose_inputs[idx].setText(f"{new_val:.1f}")
        self.lbl_status.setText(f"Status: Adjusted axis {idx+1} by {direction*delta} mm")

    def move_to_input_pose(self):
        try:
            target_vals = [float(self.pose_inputs[i].text()) for i in range(6)]
            self.send_pose_to_moveit(target_vals)
        except: pass

    def send_pose_to_moveit(self, vals):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = vals[0]/1000, vals[1]/1000, vals[2]/1000
        q = R.from_euler('xyz', vals[3:], degrees=True).as_quat()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        self.pub_pose_target.publish(msg)

    def publish_angles(self):
        msg = Float64MultiArray(); msg.data = [float(a) for a in self.current_angles]
        self.pub_angles.publish(msg)

    def save_memory(self, slot):
        self.memory[slot] = list(self.actual_angles_backup)
        self.pose_memory[slot] = list(self.current_pose) # 현재 좌표도 함께 백업
        for i in range(6): 
            self.mem_labels[slot][i].setText(f"{self.memory[slot][i]:.1f}")
            self.pose_mem_labels[slot][i].setText(f"{self.pose_memory[slot][i]:.1f}")
        self.save_all_to_csv()
        self.lbl_status.setText(f"Status: Slot {slot} Saved!")

    def move_memory(self, slot):
        self.current_angles = list(self.memory[slot]); self.publish_angles()
        self.lbl_status.setText(f"Status: Moving to Joint Pos {slot}")

    def move_pose_memory(self, slot):
        self.send_pose_to_moveit(self.pose_memory[slot])
        self.lbl_status.setText(f"Status: MoveIt Moving to Pose {slot}")

    def send_servo(self, s): msg = Bool(); msg.data = s; self.pub_servo.publish(msg)
    def control_gripper(self, s): msg = Int32(); msg.data = s; self.pub_gripper.publish(msg)
    def go_home(self): self.current_angles = [0.0]*6; self.publish_angles()

    def start_camera(self):
        if not self.is_streaming: 
            self.is_streaming = True
            self.lbl_camera_status.setText("🟢 Camera: Connected")
            self.lbl_camera_status.setStyleSheet("color: #51CF66; font-weight: bold; padding: 8px; background-color: #1e1e1e; border-radius: 3px;")
            threading.Thread(target=self.udp_receiver_thread, daemon=True).start()
    
    def stop_camera(self): 
        self.is_streaming = False
        self.lbl_camera_status.setText("🔴 Camera: Disconnected")
        self.lbl_camera_status.setStyleSheet("color: #FF6B6B; font-weight: bold; padding: 8px; background-color: #1e1e1e; border-radius: 3px;")
    def udp_receiver_thread(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sock.bind(("0.0.0.0", self.UDP_PORT)); sock.settimeout(2.0)
        while self.is_streaming:
            try:
                data, _ = sock.recvfrom(65507); nparr = np.frombuffer(data, np.uint8); frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); self.latest_frame = frame
                    qimg = QImage(frame.data, frame.shape[1], frame.shape[0], frame.shape[1]*3, QImage.Format.Format_RGB888); self.comm.update_cam.emit(qimg)
            except: continue
        sock.close()
    def set_image(self, q): self.cam_view.setPixmap(QPixmap.fromImage(q))
    def capture_image(self):
        if self.latest_frame is not None:
            cv2.imwrite(f"cap_{time.strftime('%H%M%S')}.jpg", cv2.cvtColor(self.latest_frame, cv2.COLOR_RGB2BGR))

    def save_all_to_csv(self):
        with open(self.config_file, 'w', newline='') as f:
            writer = csv.writer(f)
            for s in range(1, 6): writer.writerow(self.memory[s] + self.pose_memory[s])

    def auto_load_at_startup(self):
        if os.path.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                r = csv.reader(f)
                for i, row in enumerate(r):
                    if i < 5: 
                        self.memory[i+1] = [float(v) for v in row[:6]]
                        self.pose_memory[i+1] = [float(v) for v in row[6:12]]
                        for j in range(6): 
                            self.mem_labels[i+1][j].setText(f"{self.memory[i+1][j]:.1f}")
                            self.pose_mem_labels[i+1][j].setText(f"{self.pose_memory[i+1][j]:.1f}")

    def start_auto_process(self): threading.Thread(target=self.auto_sequence_worker, daemon=True).start()
    def auto_sequence_worker(self):
        for i in range(1, 6):
            self.current_angles = list(self.memory[i]); self.publish_angles(); time.sleep(2.0)
        self.go_home()

    def check_robot_connection(self):
        """Check if robot (arm_driver) is still connected based on encoder data reception"""
        current_time = time.time()
        time_since_last_data = current_time - self.last_encoder_time
        
        # If no data received within timeout, mark as disconnected
        if time_since_last_data > self.connection_timeout:
            if self.robot_connected:
                self.robot_connected = False
                self.lbl_robot_status.setText("🔴 Robot: Disconnected")
                self.lbl_robot_status.setStyleSheet("color: #FF6B6B; font-weight: bold; padding: 5px; background-color: #1e1e1e; border-radius: 3px;")
                self.get_logger().warn("Robot connection lost!")
        else:
            if not self.robot_connected:
                self.robot_connected = True
                self.lbl_robot_status.setText("🟢 Robot: Connected")
                self.lbl_robot_status.setStyleSheet("color: #51CF66; font-weight: bold; padding: 5px; background-color: #1e1e1e; border-radius: 3px;")
                self.get_logger().info("Robot connection established!")

def main(args=None):
    rclpy.init(args=args); app = QApplication(sys.argv); dash = JetCobotMasterDashboard()
    threading.Thread(target=rclpy.spin, args=(dash,), daemon=True).start(); sys.exit(app.exec())
if __name__ == '__main__': main()