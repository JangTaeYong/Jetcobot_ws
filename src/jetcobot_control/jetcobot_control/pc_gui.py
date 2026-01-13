import sys
import rclpy
import csv
import os
import threading
import socket
import cv2
import numpy as np
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Int32
from PyQt6.QtWidgets import (QApplication, QWidget, QGridLayout, QPushButton, 
                             QLabel, QComboBox, QGroupBox, QHBoxLayout, QVBoxLayout)
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap

# --- UI와 ROS2/UDP 스레드 간 통신을 위한 시그널 클래스 ---
class Communicate(QObject):
    update_ui = pyqtSignal(list)    # 로봇 각도 업데이트
    update_cam = pyqtSignal(QImage) # 카메라 영상 업데이트
    update_fps = pyqtSignal(str)    # FPS 텍스트 업데이트

class JetCobotMasterDashboard(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'pc_dashboard_node')
        QWidget.__init__(self)
        
        # 1. 설정 및 고정 경로
        self.config_file = os.path.join(os.getcwd(), "positions.csv")
        self.UDP_PORT = 9505
        self.current_angles = [0.0] * 6
        self.memory = {1: [0.0]*6, 2: [0.0]*6, 3: [0.0]*6}
        self.is_streaming = False

        # 2. ROS2 통신 설정
        self.pub_angles = self.create_publisher(Float64MultiArray, 'target_angles', 10)
        self.pub_servo = self.create_publisher(Bool, 'servo_status', 10)
        self.pub_gripper = self.create_publisher(Int32, 'gripper_control', 10)
        self.sub_current = self.create_subscription(
            Float64MultiArray, 'current_angles', self.receive_callback, 10)
        
        # 3. 시그널 연결
        self.comm = Communicate()
        self.comm.update_ui.connect(self.update_display)
        self.comm.update_cam.connect(self.set_image)
        self.comm.update_fps.connect(lambda fps: self.lbl_fps.setText(f"FPS: {fps}"))
        
        self.init_ui()
        self.auto_load_at_startup()

    def init_ui(self):
        self.setWindowTitle('JetCobot Master Dashboard - UDP Vision Integrated')
        self.setMinimumWidth(1450)
        master_layout = QHBoxLayout()

        # --- [좌측] 카메라 영역 ---
        vision_group = QGroupBox("Camera Vision")
        vision_layout = QVBoxLayout()
        self.cam_view = QLabel("UDP Video Waiting...")
        self.cam_view.setFixedSize(640, 480)
        self.cam_view.setStyleSheet("background-color: black; color: #00FF00; border: 3px solid #333; font-weight: bold;")
        self.cam_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        cam_ctrl_layout = QHBoxLayout()
        self.lbl_fps = QLabel("FPS: 0.0")
        self.lbl_fps.setStyleSheet("color: black; font-weight: bold; font-size: 14px;")
        self.btn_cam_connect = QPushButton("Camera CONNECT")
        self.btn_cam_disconnect = QPushButton("DISCONNECT")
        self.btn_cam_connect.clicked.connect(self.start_camera)
        self.btn_cam_disconnect.clicked.connect(self.stop_camera)
        
        cam_ctrl_layout.addWidget(self.lbl_fps); cam_ctrl_layout.addWidget(self.btn_cam_connect); cam_ctrl_layout.addWidget(self.btn_cam_disconnect)
        vision_layout.addWidget(self.cam_view); vision_layout.addLayout(cam_ctrl_layout)
        vision_group.setLayout(vision_layout)

        # --- [우측] 로봇 제어 영역 ---
        robot_layout = QVBoxLayout()
        sys_group = QGroupBox("Robot System Control")
        sys_h_layout = QHBoxLayout()
        self.btn_on = QPushButton("Servo ON"); self.btn_off = QPushButton("Servo OFF")
        self.btn_on.clicked.connect(lambda: self.send_servo(True))
        self.btn_off.clicked.connect(lambda: self.send_servo(False))
        self.btn_home = QPushButton("HOME")
        self.btn_home.setStyleSheet("background-color: #FFECB3; color: black; font-weight: bold;")
        self.btn_home.clicked.connect(self.go_home)
        self.btn_grip = QPushButton("GRIP"); self.btn_ungrip = QPushButton("UNGRIP")
        self.btn_grip.clicked.connect(lambda: self.control_gripper(1))
        self.btn_ungrip.clicked.connect(lambda: self.control_gripper(0))
        
        self.speed_combo = QComboBox(); self.speed_combo.addItems(["Speed: 20", "Speed: 50", "Speed: 80"])
        self.step_combo = QComboBox(); self.step_combo.addItems(["Step: 1", "Step: 5", "Step: 10"])
        
        for w in [self.btn_on, self.btn_off, self.btn_home, self.btn_grip, self.btn_ungrip]: sys_h_layout.addWidget(w)
        sys_h_layout.addStretch(); sys_h_layout.addWidget(self.speed_combo); sys_h_layout.addWidget(self.step_combo)
        sys_group.setLayout(sys_h_layout)

        # 중앙 각도 그리드 (순서 뒤집기 적용)
        grid = QGridLayout(); grid.setSpacing(10)
        headers = ["Axis", "Jog (+/-)", "Actual Ang", "Memory Pos 1", "Memory Pos 2", "Memory Pos 3"]
        for col, text in enumerate(headers):
            lbl = QLabel(f"<b>{text}</b>"); lbl.setAlignment(Qt.AlignmentFlag.AlignCenter); grid.addWidget(lbl, 0, col)

        # 리스트 미리 확보
        self.enc_labels = [None] * 6
        self.mem_labels = {1: [None]*6, 2: [None]*6, 3: [None]*6}

        # i=0(Joint 1)이 맨 아래(row 6), i=5(Joint 6)이 맨 위(row 1)
        for i in range(6):
            row = 6 - i 
            grid.addWidget(QLabel(f"Joint {i+1}"), row, 0)
            
            jog_box = QHBoxLayout()
            btn_m = QPushButton("-"); btn_p = QPushButton("+")
            btn_m.clicked.connect(lambda ch, idx=i: self.jog(idx, -1))
            btn_p.clicked.connect(lambda ch, idx=i: self.jog(idx, 1))
            jog_box.addWidget(btn_m); jog_box.addWidget(btn_p)
            grid.addLayout(jog_box, row, 1)
            
            enc_lbl = QLabel("0.0")
            enc_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            enc_lbl.setStyleSheet("border: 1px solid gray; background-color: white; color: black; font-weight: bold; font-size: 14px;")
            self.enc_labels[i] = enc_lbl
            grid.addWidget(enc_lbl, row, 2)
            
            for m_idx in range(1, 4):
                m_lbl = QLabel("---")
                m_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
                m_lbl.setStyleSheet("border: 1px solid silver; background-color: #fdfdfd; color: black; font-weight: bold;")
                self.mem_labels[m_idx][i] = m_lbl
                grid.addWidget(m_lbl, row, m_idx + 2)

        for m_idx in range(1, 4):
            vbox = QVBoxLayout()
            btn_s = QPushButton(f"Pos {m_idx} 저장"); btn_g = QPushButton(f"Pos {m_idx} 이동")
            btn_s.clicked.connect(lambda ch, m=m_idx: self.save_memory(m))
            btn_g.clicked.connect(lambda ch, m=m_idx: self.move_memory(m))
            vbox.addWidget(btn_s); vbox.addWidget(btn_g)
            grid.addLayout(vbox, 7, m_idx + 2)

        robot_layout.addWidget(sys_group); robot_layout.addLayout(grid); robot_layout.addStretch()
        master_layout.addWidget(vision_group); master_layout.addLayout(robot_layout)
        self.setLayout(master_layout)
        self.show()

    # --- 카메라 UDP 로직 ---
    def start_camera(self):
        if not self.is_streaming:
            self.is_streaming = True
            threading.Thread(target=self.udp_receiver_thread, daemon=True).start()
            self.btn_cam_connect.setEnabled(False)

    def stop_camera(self):
        self.is_streaming = False
        self.btn_cam_connect.setEnabled(True)
        self.cam_view.setText("Camera Disconnected")

    def udp_receiver_thread(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.UDP_PORT))
        sock.settimeout(2.0)
        prev_time = time.time()
        while self.is_streaming:
            try:
                data, _ = sock.recvfrom(65507)
                curr_time = time.time()
                diff = curr_time - prev_time
                if diff > 0: self.comm.update_fps.emit(f"{1.0 / diff:.1f}")
                prev_time = curr_time
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = frame.shape
                    qimg = QImage(frame.data, w, h, w * ch, QImage.Format.Format_RGB888)
                    self.comm.update_cam.emit(qimg)
            except: continue
        sock.close()

    def set_image(self, qimg): self.cam_view.setPixmap(QPixmap.fromImage(qimg))

    # --- 로봇 제어 로직 ---
    def auto_load_at_startup(self):
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    reader = csv.reader(f)
                    for i, row in enumerate(reader):
                        if i < 3:
                            slot = i + 1
                            self.memory[slot] = [float(v) for v in row]
                            for j in range(6): self.mem_labels[slot][j].setText(f"{self.memory[slot][j]:.1f}")
            except: pass

    def save_memory(self, slot):
        self.memory[slot] = list(self.current_angles)
        for i in range(6):
            self.mem_labels[slot][i].setText(f"{self.memory[slot][i]:.1f}")
            self.mem_labels[slot][i].setStyleSheet("border: 2px solid #2196F3; background-color: #E3F2FD; color: black; font-weight: bold;")
        try:
            with open(self.config_file, 'w', newline='') as f:
                writer = csv.writer(f)
                for s in range(1, 4): writer.writerow(self.memory[s])
        except Exception as e: self.get_logger().error(f"Save failed: {e}")

    def move_memory(self, slot):
        self.current_angles = list(self.memory[slot])
        self.publish_angles()

    def go_home(self):
        self.current_angles = [0.0] * 6
        self.publish_angles()

    def control_gripper(self, state):
        msg = Int32(); msg.data = state
        self.pub_gripper.publish(msg)

    def jog(self, idx, direction):
        try:
            step = float(self.step_combo.currentText().split(": ")[1])
            self.current_angles[idx] += (direction * step)
            self.publish_angles()
        except: pass

    def send_servo(self, state):
        msg = Bool(); msg.data = state
        self.pub_servo.publish(msg)

    def publish_angles(self):
        msg = Float64MultiArray()
        msg.data = [float(a) for a in self.current_angles]
        self.pub_angles.publish(msg)

    def receive_callback(self, msg): self.comm.update_ui.emit(list(msg.data))

    def update_display(self, angles):
        for i in range(6):
            self.enc_labels[i].setText(f"{angles[i]:.1f}")
            self.current_angles[i] = angles[i]

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    dashboard = JetCobotMasterDashboard()
    ros_thread = threading.Thread(target=rclpy.spin, args=(dashboard,), daemon=True)
    ros_thread.start()
    try: sys.exit(app.exec())
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()