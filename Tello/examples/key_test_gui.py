import sys
import threading
import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget)
from PyQt5.QtGui import QPalette, QPixmap, QBrush, QImage
from PyQt5.QtCore import Qt, QTimer
from djitellopy import Tello
import time
from exampleTello import go_to_aruco_and_land

# ArUco tracking setup
drone = Tello()
aruco_tracking = False
pError = 0
SPEED = 20
DISTANCE_THRESHOLD = 30
LANDING_THRESHOLD = 10000
PID = [1.0, 0.0, 0.0]  # PID constants for yaw control

class TelloGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tello Drone GUI")
        self.setGeometry(100, 100, 800, 600)

        drone.connect()
        drone.streamon()

        self.track_marker = False
        self.find_marker = False
        self.camera_down = False  # Initialize camera state

        self.setFocusPolicy(Qt.StrongFocus)  # To ensure key events are received

        self.initUI()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery)
        self.battery_timer.start(5000)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

    def initUI(self):
        self.set_background("Images/Gold-Brayer2.png")

        self.video_label = QLabel(self)
        self.video_label.setFixedSize(640, 480)

        self.battery_label = QLabel(f"Battery: {drone.get_battery()}%", self)
        self.battery_label.setAlignment(Qt.AlignCenter)
        
        self.toggle_camera_button = QPushButton("Switch Camera")
        self.toggle_camera_button.clicked.connect(self.toggle_camera)

        self.takeoff_button = QPushButton("Take Off")
        self.takeoff_button.clicked.connect(self.takeoff)

        self.land_button = QPushButton("Land")
        self.land_button.clicked.connect(self.land)

        # self.up_button = QPushButton("Up")
        # self.up_button.clicked.connect(lambda: drone.move_up(30))

        # self.down_button = QPushButton("Down")
        # self.down_button.clicked.connect(lambda: drone.move_down(30))

        # self.left_button = QPushButton("Left")
        # self.left_button.clicked.connect(lambda: drone.move_left(30))

        # self.right_button = QPushButton("Right")
        # self.right_button.clicked.connect(lambda: drone.move_right(30))

        # self.forward_button = QPushButton("Forward")
        # self.forward_button.clicked.connect(lambda: drone.move_forward(30))

        # self.backward_button = QPushButton("Backward")
        # self.backward_button.clicked.connect(lambda: drone.move_back(30))

        # self.rotate_cw_button = QPushButton("Rotate CW")
        # self.rotate_cw_button.clicked.connect(lambda: drone.rotate_clockwise(30))

        # self.rotate_ccw_button = QPushButton("Rotate CCW")
        # self.rotate_ccw_button.clicked.connect(lambda: drone.rotate_counter_clockwise(30))

        self.track_button = QPushButton("Track Marker")
        self.track_button.clicked.connect(self.toggle_track)

        control_layout = QVBoxLayout()
        control_layout.addWidget(self.battery_label)
        control_layout.addWidget(self.toggle_camera())
        control_layout.addWidget(self.takeoff_button)
        control_layout.addWidget(self.land_button)
        # control_layout.addWidget(self.up_button)
        # control_layout.addWidget(self.down_button)
        # control_layout.addWidget(self.left_button)
        # control_layout.addWidget(self.right_button)
        # control_layout.addWidget(self.forward_button)
        # control_layout.addWidget(self.backward_button)
        # control_layout.addWidget(self.rotate_cw_button)
        # control_layout.addWidget(self.rotate_ccw_button)
        control_layout.addWidget(self.track_button)

        main_layout = QHBoxLayout()
        main_layout.addWidget(self.video_label)
        main_layout.addLayout(control_layout)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def set_background(self, image_path):
        palette = QPalette()
        pixmap = QPixmap(image_path)
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.setPalette(palette)

    def update_frame(self):
        frame = drone.get_frame_read().frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if self.find_marker and ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if self.track_marker and ids is not None:
            self.track_aruco(ids, corners, frame)

        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qt_image))

    def update_battery(self):
        battery = drone.get_battery()
        self.battery_label.setText(f"Battery: {battery}%")

    def toggle_camera(self):
            try:
                if self.camera_down:
                    drone.set_video_direction(1)  # Front camera
                    print("Switched to front camera")
                else:
                    drone.set_video_direction(0)  # Downward camera
                    print("Switched to downward camera")
                self.camera_down = not self.camera_down
            except Exception as e:
                print(f"Error toggling camera: {e}")


    def takeoff(self):
        drone.takeoff()

    def land(self):
        drone.land()

    def toggle_track(self):
        self.track_marker = not self.track_marker
        print(f"Track marker {'enabled' if self.track_marker else 'disabled'}")
    
    def start_landing_thread(self):
        self.thread = threading.Thread(target=self.safe_go_to_aruco_and_land, daemon=True)
        self.thread.start()

    def safe_go_to_aruco_and_land(self):
        try:
            go_to_aruco_and_land()  # Your original logic
        except Exception as e:
            print(f"Landing failed: {e}")

    def closeEvent(self, event):
        drone.end()
        event.accept()

    def keyPressEvent(self, event):
        key = event.key()

        if key == Qt.Key_W:
            drone.send_rc_control(0, 30, 0, 0)  # Forward
        elif key == Qt.Key_S:
            drone.send_rc_control(0, -30, 0, 0)  # Backward
        elif key == Qt.Key_A:
            drone.send_rc_control(-30, 0, 0, 0)  # Left
        elif key == Qt.Key_D:
            drone.send_rc_control(30, 0, 0, 0)  # Right
        elif key == Qt.Key_Up:
            drone.send_rc_control(0, 0, 30, 0)  # Up
        elif key == Qt.Key_Down:
            drone.send_rc_control(0, 0, -30, 0)  # Down
        elif key == Qt.Key_E:
            drone.send_rc_control(0, 0, 0, 30)  # Rotate clockwise
        elif key == Qt.Key_Q:
            drone.send_rc_control(0, 0, 0, -30)  # Rotate counter-clockwise
        elif key == Qt.Key_C:
            self.toggle_camera()  # Camera toggle on 'C' key

    def keyReleaseEvent(self, event):
        drone.send_rc_control(0, 0, 0, 0)  # Stop all movement when key is released

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = TelloGUI()
    gui.show()
    sys.exit(app.exec_())
