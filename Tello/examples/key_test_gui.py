import sys
import threading
import cv2
import cv2.aruco as aruco
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget)
from PyQt5.QtGui import QPalette, QPixmap, QBrush, QImage
from PyQt5.QtCore import Qt, QTimer
from djitellopy import Tello
import time

# ArUco tracking setup
drone = Tello()
aruco_tracking = False
pError = 0
camera_down = False
face_tracking = False
body_tracking = False
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
aruco_params = aruco.DetectorParameters()

# Constants
SPEED = 60
PID = [0.3, 0.0005, 0.1]  # Reduced proportional gain
LANDING_THRESHOLD = 1500  # Increased to avoid premature landing
DISTANCE_THRESHOLD = 20  # Distance threshold for moving forward

# ArUco detection function
def find_aruco(img):
    global aruco_tracking

    if img is None or img.size == 0:
        print("Warning: Received an empty frame in find_aruco.")
        return [0, 0], 0, np.zeros((480, 640, 3), dtype=np.uint8)  # Return a blank image instead

    if not aruco_tracking:
        return [0, 0], 0, img  # Return default values when not tracking

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    # Process detected markers
    if ids is not None and len(ids) > 0:
        # Draw detected markers
        aruco.drawDetectedMarkers(img, corners, ids)

        # Get the center of the first detected marker
        c = corners[0][0]
        cx = int(np.mean(c[:, 0]))
        cy = int(np.mean(c[:, 1]))

        # Calculate area by finding the area of the quadrilateral
        area = cv2.contourArea(c.astype(np.float32))

        # Mark center of marker
        cv2.circle(img, (cx, cy), 5, (0, 255, 255), cv2.FILLED)

        return [cx, cy], area, img
    else:
        return [0, 0], 0, img  # No marker detected

# Tracking function for ArUco
def track_aruco(info, width, pid, p_error):
    global aruco_tracking

    if not aruco_tracking or info[0] == 0:
        return 0, p_error  # Don't track if disabled or no marker

    # Calculate error from center
    error = info[0] - width // 2

    # PID calculation
    p = pid[0] * error
    i = pid[1] * (error + p_error)
    d = pid[2] * (error - p_error)

    # Calculate yaw velocity
    yaw_velocity = int(p + i + d)

    # Limit yaw velocity
    yaw_velocity = max(min(yaw_velocity, SPEED), -SPEED)

    return yaw_velocity, error

# Function to approach and land on the ArUco marker
def go_to_aruco_and_land():
    global aruco_tracking, pError
    aruco_tracking = True  # Enable ArUco tracking
    last_seen_time = time.time()
    landed = False
    marker_lost = False

    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)

    while not landed:
        try:
            # Get the frame from the drone's camera
            img = drone.get_frame_read().frame
            if img is None or img.size == 0:
                print("Warning: Empty frame received from drone. Skipping...")
                continue
                
            frame = cv2.resize(img, (640, 480))
            aruco_info, area, processed_frame = find_aruco(frame)

            # Marker tracking logic
            if aruco_info[0] != 0:
                last_seen_time = time.time()
                cx, cy = aruco_info
                error_x = cx - frame.shape[1] // 2
                error_y = cy - frame.shape[0] // 2

                # PID calculations
                yaw_velocity, pError = track_aruco(aruco_info, frame.shape[1], PID, pError)
                
                # Forward control
                forward_speed = max(20, int(SPEED * (1 - min(area / (LANDING_THRESHOLD * 1.5), 1))))
                
                # Vertical adjustment based on vertical position
                vertical_speed = int(PID[0] * error_y)
                
                # Only move forward if marker is roughly centered (error within threshold)
                if abs(error_x) > DISTANCE_THRESHOLD:
                    drone.send_rc_control(0, forward_speed, -vertical_speed, yaw_velocity)
                    time.sleep(1)
                elif abs(error_x) < DISTANCE_THRESHOLD:
                    drone.send_rc_control(0, forward_speed, -vertical_speed, yaw_velocity)
                    time.sleep(1)
                    
                print(f"Area: {area}, Forward: {forward_speed}, Vertical: {vertical_speed}, Yaw: {yaw_velocity}")
            else:
                # Lost marker handling
                if time.time() - last_seen_time > 2 and not marker_lost:  # 2 seconds without marker
                    marker_lost = True  # Prevent re-execution
                    print("Marker lost - hovering")
                    drone.send_rc_control(0, 0, 0, 0)
                    drone.send_rc_control(0, 40, 0, 0)
                    drone.land()
                    # Hover in place
                    drone.send_rc_control(0, 0, 0, 0)
                    time.sleep(1)

                    # Hover in place and try a slight descent before switching
                    drone.send_rc_control(0, 0, 30, 0)  # Gradually ascend
                    time.sleep(2)

                    # Re-check with front camera before switching
                    img = drone.get_frame_read().frame
                    aruco_info, area, processed_frame = find_aruco(img)
                    
                    # Marker still not found? Switch cameras.
                    if aruco_info[0] == 0:
                        print("Switching to bottom camera...")
                        # drone.set_video_direction(drone.CAMERA_DOWNWARD)
                        processed_frame = bottom_camera_and_land(img)

        except Exception as e:
            print(f"Error during ArUco tracking: {e}")
            drone.land()
            break  # Break out of the loop to prevent infinite errors

    pError = 0  # Reset PID error
    aruco_tracking = False
    cv2.destroyAllWindows()

def search_for_aruco_bottom_camera():
    global aruco_tracking

    search_step = 20  # Degrees to rotate at a time
    search_distance = 20  # Distance to move forward
    max_attempts = 2  # Limit retries to prevent infinite search loops
    attempt = 0

    # Loop to search for the ArUco marker with a limited number of attempts
    while attempt < max_attempts:
        try:
            # Get the frame from the bottom camera
            frame = drone.get_frame_read().frame
            if frame is None or frame.size == 0:
                print("Warning: Empty frame received from drone. Skipping...")
                continue  # Skip to the next iteration if the frame is empty

            # Resize the frame for processing
            frame = cv2.resize(frame, (640, 480))
            aruco_info, area, processed_frame = find_aruco(frame)  # Find ArUco markers in the frame

            # If a marker is found, return its information
            if aruco_info[0] != 0:
                print("Marker found!")
                drone.land()  # Command the drone to land
                cv2.destroyAllWindows()  # Close any OpenCV windows
                
                return aruco_info, area

            # If no marker is found, rotate and move in a search pattern
            print("Marker not found, searching...")
            print("Moving forward...")
            drone.send_rc_control(0, 1, 0, 0)  # Move forward
            time.sleep(1)  # Wait for the movement to complete
        except Exception as e:
            print(f"Error during search: {e}")  # Handle any exceptions that occur during the search
        
        attempt += 1  # Increment the attempt counter

    print("Max search attempts reached. Returning to hover mode.")
    return aruco_info, area  # Return the last known ArUco info and area

def check_imu():
    """Helper function to check if the IMU is valid before rotating."""
    try:
        altitude = drone.get_height()  # Get the drone's altitude
        print(f"IMU check - Altitude: {altitude}")
        return altitude is not None  # Ensure a valid response from the IMU
    except Exception as e:
        print(f"IMU check failed: {e}")  # Handle any exceptions during the IMU check
    return False  # Return False if the check fails

# Simplified function to approach and land on the ArUco marker using the bottom camera
def bottom_camera_and_land(frame):
    global aruco_tracking, pError
    aruco_tracking = True  # Enable ArUco tracking
    last_seen_time = time.time()  # Record the last time the marker was seen
    landed = False  # Flag to indicate if the drone has landed
    try:
        # Get the frame from the drone's bottom camera
        frame = drone.get_frame_read().frame
        if frame is None or frame.size == 0:
            print("Warning: Empty frame received from drone. Skipping...")
            # Skip to the next iteration if the frame is empty

        # Resize the frame for processing
        frame = cv2.resize(frame, (640, 480))
        aruco_info, area, processed_frame = find_aruco(frame)  # Find ArUco markers in the frame

        # Marker tracking logic
        if aruco_info[0] != 0:  # If the marker is found
            print("Marker found, landing...")
            drone.land()
            landed = True  # Set the landed flag to True
            drone.end()
        
        elif aruco_info[0] == 0:
            # If the marker is lost, start searching for it
            print("Marker not found, starting search...")
            aruco_info, area = search_for_aruco_bottom_camera()  # Call the search function to find the marker

        print("show vid")  # Placeholder for showing video feed

    except Exception as e:
        print(f"Error updating GUI line 369: {e}")  # Handle any exceptions that occur during the process
        drone.land()  # Command the drone to land in case of an error
        drone.end()# Break out of the loop to prevent infinite errors

    pError = 0  # Reset PID error for future use
    aruco_tracking = False  # Disable ArUco tracking after landing

# GUI Class
class TelloGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Controller")
        self.setGeometry(500, 250, 850, 650)

        self.setFocusPolicy(Qt.StrongFocus)
        self.set_background("Images/Gold-Brayer2.png")  # <-- Change to your image path

        drone.connect()
        drone.streamon()

        self.track_marker = False
        self.find_marker = False
        self.camera_down = False  # Initialize camera state

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery)
        self.battery_timer.start(100)

        self.temp_timer = QTimer()
        self.temp_timer.timeout.connect(self.update_temp)
        self.temp_timer.start(100)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.initUI()

    def initUI(self):
        # Central widget and main layout (VERTICAL now)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()  # Vertical layout for everything
        central_widget.setLayout(main_layout)
        main_layout.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        # --- Video and Temperature ---
        battery = self.update_battery
        self.battery_label = QLabel(f"{battery}")
        self.battery_label.setAlignment(Qt.AlignCenter)
        self.battery_label.setStyleSheet("font-size: 16px; color: white;")

        temp = self.update_temp
        self.temp_label = QLabel(f"{temp}°C")
        self.temp_label.setAlignment(Qt.AlignCenter)
        self.temp_label.setStyleSheet("font-size: 16px; color: white;")

        # --- Switch Camera Button ---
        self.switch_camera_button = QPushButton("Switch Camera")
        self.switch_camera_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: #9b59b6; color: white;")
        self.switch_camera_button.setFixedSize(160, 35)
        self.switch_camera_button.clicked.connect(self.toggle_camera)

        switch_button_layout = QHBoxLayout()
        switch_button_layout.setAlignment(Qt.AlignCenter)
        switch_button_layout.addWidget(self.switch_camera_button)

        # --- Video Feed Placeholder ---
        self.video_label = QLabel(self)
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 16px;")
        self.video_label.setAlignment(Qt.AlignCenter)

        # --- Spacer ---
        spacer = QLabel("")
        spacer.setFixedHeight(10)

        main_layout.addWidget(self.battery_label)
        main_layout.addWidget(self.temp_label)
        main_layout.addLayout(switch_button_layout)
        main_layout.addWidget(self.video_label)
        main_layout.addWidget(spacer)

        # --- Control Buttons (Horizontally below video) ---
        button_layout = QHBoxLayout()
        button_layout.setAlignment(Qt.AlignCenter)

        self.takeoff_button = QPushButton("Take Off")
        self.takeoff_button.clicked.connect(self.takeoff)
        self.land_button = QPushButton("Land")
        self.land_button.clicked.connect(self.land)
        self.track_button = QPushButton("Go To ArUco")
        self.track_button.clicked.connect(self.start_landing_thread)
        self.emergency_button = QPushButton("Emergency Shut Off")
        self.emergency_button.clicked.connect(drone.emergency)

        # Set individual colors
        self.takeoff_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: green; color: white;")
        self.land_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: #f1c40f; color: white;")
        self.track_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: #3498db; color: white;")
        self.emergency_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: red; color: white;")

        for btn in [self.takeoff_button, self.land_button, self.track_button, self.emergency_button]:
            btn.setFixedSize(160, 40)
            button_layout.addWidget(btn)

        # Add the button layout to the main layout
        main_layout.addLayout(button_layout)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def set_background(self, image_path):
        palette = QPalette()
        pixmap = QPixmap(image_path)
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.setPalette(palette)

    def update_frame(self):
        try:
            # Get frame
            frame_read = drone.get_frame_read()
            if frame_read is None:
                print("Failed to get frame read object.")
                return
            
            img = frame_read.frame
            if img is None or img.size == 0:
                print("Warning: Empty frame received from drone. Skipping...")
                return

            # Resize and process
            img = cv2.resize(img, (640, 480))
            aruco_info, area, processed_frame = find_aruco(img)  # This will draw markers if tracking is on

            # Rotate if camera is facing down
            
            # Convert to RGB
            rgb_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)

            # Convert to QImage and display
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qt_image))

        except Exception as e:
            print(f"in update_frame: {e}")

    def update_battery(self):
        try:
            battery = drone.get_battery()
            self.battery_label.setText(f"Battery: {battery}%")
        except Exception as e:
            print(f"Error getting battery status: {e}")
            drone.end()

    def update_temp(self):
        try:
            temp = drone.get_temperature()
            self.temp_label.setText(f"Temperature: {temp}°C")
        except Exception as e:
            print(f"Error getting temperature: {e}")
            drone.end()

    def toggle_camera(self):
        global camera_down
        try:
            if self.camera_down:
                camera_down = False
                drone.set_video_direction(drone.CAMERA_FORWARD)  # Front camera
                print("Switched to front camera")
            else:
                camera_down = True
                drone.set_video_direction(drone.CAMERA_DOWNWARD)  # Downward camera
                print("Switched to downward camera")
            self.camera_down = not self.camera_down
        except Exception as e:
            print(f"Error toggling camera: {e}")

    def takeoff(self):
        try:
            drone.takeoff()
            drone.send_rc_control(0, 0, 0, 0)  # Stop all movement
        except Exception as e:
            print(f"Error during takeoff: {e}")

    def land(self):
        try:
            drone.land()
        except Exception as e:
            print(f"Error during landing: {e}")

    def start_landing_thread(self):
        self.thread = threading.Thread(target=self.safe_go_to_aruco_and_land, daemon=True)
        self.thread.start()

    def safe_go_to_aruco_and_land(self):
        try:
            print("Starting ArUco landing sequence...")
            go_to_aruco_and_land()
            print("ArUco landing sequence complete.")
        except Exception as e:
            print(f"Landing failed: {e}")
        finally:
            try:
                drone.send_rc_control(0, 0, 0, 0)
                drone.end()
            except Exception as e:
                print(f"[WARN] Cleanup error: {e}")

    def closeEvent(self, event):
        try:
            drone.land()
            drone.end()
        except Exception as e:
            print(f"[WARN] on close: {e}")
        event.accept()

    def keyPressEvent(self, event):
        key = event.key()

        if key == Qt.Key_W:
            drone.send_rc_control(0, 30, 0, 0)  # Forward
        elif key == Qt.Key_S:
            drone.send_rc_control(0, -30, 0, 0)  # Backward
        elif key == Qt.Key_A:
            drone.send_rc_control(30, 0, 0, 0)  # Left
        elif key == Qt.Key_D:
            drone.send_rc_control(-30, 0, 0, 0)  # Right
        elif key == Qt.Key_Up:
            drone.send_rc_control(0, 0, 30, 0)  # Up
        elif key == Qt.Key_Down:
            drone.send_rc_control(0, 0, -30, 0)  # Down
        elif key == Qt.Key_E:
            drone.send_rc_control(0, 0, 0, 30) # Rotate clockwise
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
