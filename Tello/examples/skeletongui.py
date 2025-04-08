import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget)
from PyQt5.QtGui import QPalette, QPixmap, QBrush
from PyQt5.QtCore import Qt

class KeyboardVideoDemo(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Keyboard + Video + Controls Demo")
        self.setGeometry(500, 250, 850, 650)

        self.setFocusPolicy(Qt.StrongFocus)
        self.set_background("Images/Gold-Brayer2.png")  # <-- Change to your image path

        self.init_ui()

    def set_background(self, image_path):
        palette = QPalette()
        pixmap = QPixmap(image_path)
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.setPalette(palette)

    def init_ui(self):
        # Central widget and main layout (VERTICAL now)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()  # Vertical layout for everything
        central_widget.setLayout(main_layout)
        main_layout.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        # --- Video and Temperature ---
        self.battery_label = QLabel("Battery: --%")
        self.battery_label.setAlignment(Qt.AlignCenter)
        self.battery_label.setStyleSheet("font-size: 16px; color: white;")

        self.temp_label = QLabel("Temp: --°C")
        self.temp_label.setAlignment(Qt.AlignCenter)
        self.temp_label.setStyleSheet("font-size: 16px; color: white;")

        # --- Switch Camera Button ---
        self.switch_camera_button = QPushButton("Switch Camera")
        self.switch_camera_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: #9b59b6; color: white;")
        self.switch_camera_button.setFixedSize(160, 35)
        self.switch_camera_button.clicked.connect(self.switch_camera)
        
        switch_button_layout = QHBoxLayout()
        switch_button_layout.setAlignment(Qt.AlignCenter)
        switch_button_layout.addWidget(self.switch_camera_button)

        # --- Video Feed Placeholder ---
        self.video_label = QLabel("Video Feed Placeholder")
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 16px;")
        self.video_label.setAlignment(Qt.AlignCenter)

        # --- Spacer ---
        spacer = QLabel("")
        spacer.setFixedHeight(10)

        # --- Status Label ---
        self.status_label = QLabel("Press any key")
        self.status_label.hide()
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 18px; color: white;")

        # --- Add Widgets to Main Layout ---
        main_layout.addWidget(self.battery_label)
        main_layout.addWidget(self.temp_label)
        main_layout.addLayout(switch_button_layout)
        main_layout.addWidget(self.video_label)
        main_layout.addWidget(spacer)
        main_layout.addWidget(self.status_label)
        
        # --- Control Buttons (Horizontally below video) ---
        button_layout = QHBoxLayout()
        button_layout.setAlignment(Qt.AlignCenter)

        self.takeoff_button = QPushButton("Take Off")
        self.land_button = QPushButton("Land")
        self.goto_aruco_button = QPushButton("Go to ArUco")
        self.emergency_button = QPushButton("Emergency Shut Off")

        # Set individual colors
        self.takeoff_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: green; color: white;")
        self.land_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: #f1c40f; color: white;")
        self.goto_aruco_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: #3498db; color: white;")
        self.emergency_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: red; color: white;")

        for btn in [self.takeoff_button, self.land_button, self.goto_aruco_button, self.emergency_button]:
            btn.setFixedSize(160, 40)
            button_layout.addWidget(btn)

        # Add the button layout to the main layout
        main_layout.addLayout(button_layout)

    def keyPressEvent(self, event):
        key_name = event.text() or f"Key code: {event.key()}"
        self.status_label.setText(f"Key Pressed: {key_name}")

    def keyReleaseEvent(self, event):
        self.status_label.setText("Key Released")

    def switch_camera(self):
        print("Camera switched!") 

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = KeyboardVideoDemo()
    window.show()
    sys.exit(app.exec_())
