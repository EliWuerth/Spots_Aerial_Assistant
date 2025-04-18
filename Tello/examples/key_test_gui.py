import sys
import threading
import cv2
import cv2.aruco as aruco
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QMessageBox)
from PyQt5.QtGui import QPalette, QPixmap, QBrush, QImage
from PyQt5.QtCore import Qt, QTimer
from djitellopy import Tello
import time

# ArUco tracking setup
drone = Tello()
aruco_tracking = False
pError = 0
camera_down = False
human_tracking = False
aruco_dict_type = aruco.DICT_6X6_50
aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
aruco_params = aruco.DetectorParameters()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Constants
SPEED = 60
PID = [0.3, 0.0005, 0.1]  # Reduced proportional gain
LANDING_THRESHOLD = 1500  # Increased to avoid premature landing
DISTANCE_THRESHOLD = 20  # Distance threshold for moving forward

def find_human(img, tracking_enabled=True):
    if img is None or not tracking_enabled:
        return [0, 0], img

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

    if len(faces) > 0:
        (x, y, w, h) = faces[0]  # Use the first detected face
        cx = x + w // 2
        cy = y + h // 2

        # Draw a rectangle and center point
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)

        return [cx, cy], img

    return [0, 0], img

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
    current_dict = aruco.getPredefinedDictionary(aruco_dict_type)
    corners, ids, _ = aruco.detectMarkers(gray, current_dict, parameters=aruco_params)
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
    global aruco_tracking, pError, camera_down
    aruco_tracking = True  # Enable ArUco tracking
    last_seen_time = time.time()  # Record the last time the marker was seen
    landed = False  # Flag to indicate if the drone has landed
    marker_lost = False  # Flag to indicate if the marker has been lost

    while not landed:
        try:
            # Get the frame from the drone's camera
            img = drone.get_frame_read().frame
            if img is None or img.size == 0:
                print("Warning: Empty frame received from drone. Skipping...")
                continue # Skip to the next iteration if the frame is empty
                
            frame = cv2.resize(img, (640, 480)) # Resize the frame for processing
            aruco_info, area, processed_frame = find_aruco(frame)  # Find ArUco markers in the frame

            # Marker tracking logic
            if aruco_info[0] != 0 and (camera_down == True):
                print("Aruco Found, Drone landing...")
                drone.land()  # Command the drone to land
                landed = True  # Set the landed flag to True
                show_landed_message()  # Show landing success message
            elif aruco_info[0] != 0:
                print("Marker Found")  # Marker detected
                last_seen_time = time.time()  # Update last seen time
                cx, cy = aruco_info  # Get the center coordinates of the marker
                error_x = cx - frame.shape[1] // 2  # Calculate horizontal error from the center
                error_y = cy - frame.shape[0] // 2  # Calculate vertical error from the center

                # PID calculations
                yaw_velocity, pError = track_aruco(aruco_info, frame.shape[1], PID, pError)
                
                # Forward control
                forward_speed = max(20, int(SPEED * (1 - min(area / (LANDING_THRESHOLD * 1.5), 1))))
                
                # Vertical adjustment based on vertical position
                vertical_speed = int(PID[0] * error_y)
                
                # Only move forward if marker is roughly centered (error within threshold)
                if abs(error_x) > DISTANCE_THRESHOLD:
                    drone.send_rc_control(0, forward_speed, -vertical_speed, yaw_velocity)  # Move the drone
                    time.sleep(1)  # Wait for a second before the next control command
                elif abs(error_x) < DISTANCE_THRESHOLD:
                    drone.send_rc_control(0, forward_speed, -vertical_speed, yaw_velocity)  # Maintain position
                    time.sleep(1)  # Wait for a second before the next control command
                    
                print(f"Area: {area}, Forward: {forward_speed}, Vertical: {vertical_speed}, Yaw: {yaw_velocity}") # Log control values
            else:
                # Handle the case where the marker is lost
                if time.time() - last_seen_time > 2 and not marker_lost:  # Check if marker has been lost for more than 2 seconds
                    marker_lost = True  # Prevent re-execution of lost marker logic
                    print("Marker lost - hovering")  # Indicate that the marker is lost

                    drone.send_rc_control(0, 0, 0, 0)  # Hover in place
                    time.sleep(1)  # Wait for a second

                    toggle_camera()  # Switch camera view
                    
                    
                    # Drone hovers while searching for the marker
                    drone.send_rc_control(0, 0, 0, 0)
                    time.sleep(2)  # Wait before attempting to land again

                    bottom_camera_and_land()  # Attempt to land using the bottom camera
        except Exception as e:
            print(f"Error during ArUco tracking: {e}")  # Log any errors encountered
            drone.end()  # End the drone session
            break  # Exit the loop

    pError = 0  # Reset PID error after landing
    aruco_tracking = False  # Disable ArUco tracking
    drone.end()  # End the drone session

# Simplified function to approach and land on the ArUco marker using the bottom camera
def bottom_camera_and_land():
    global aruco_tracking, pError
    aruco_tracking = True  # Enable ArUco tracking
    last_seen_time = time.time()  # Record the last time the marker was seen
    landed = False  # Flag to indicate if the drone has landed
    
    toggle_aruco_dict()
    # Get the frame from the drone's bottom camera
    frame = drone.get_frame_read().frame
    
    # Resize the frame for processing
    frame = cv2.resize(frame, (640, 480))
    aruco_info, area, processed_frame = find_aruco(frame)  # Find ArUco markers in the frame
    
    if aruco_info[0] == 0:
        # If the marker is lost, start searching for it
        print("Marker not found, starting search...")
        drone.send_rc_control(0,0,0,0) # Hover in place
        time.sleep(1)  # Wait for a second before searching again
    else:
        # Command the drone to land if the marker is found
        drone.land()
        show_landed_message()  # Show landing success message

# Function to toggle between front and downward cameras
def toggle_camera():
    global camera_down
    try:
        if camera_down:
            camera_down = False  # Switch to front camera
            drone.set_video_direction(drone.CAMERA_FORWARD)  # Set camera to forward view
            print("Switched to front camera")  # Log camera switch
        else:
            camera_down = True  # Switch to downward camera
            drone.set_video_direction(drone.CAMERA_DOWNWARD)  # Set camera to downward view
            print("Switched to downward camera")  # Log camera switch
        
    except Exception as e:
        print(f"Error toggling camera: {e}")  # Log any errors encountered
    return camera_down

def toggle_aruco_dict():
        global aruco_dict_type
        if aruco_dict_type == aruco.DICT_6X6_50:
            aruco_dict_type = aruco.DICT_4X4_50
            print("Switched to ArUco Dictionary: 4x4_50")
        else:
            aruco_dict_type = aruco.DICT_6X6_50
            print("Switched to ArUco Dictionary: 6x6_50")

# Function to show a message box indicating successful landing
def show_landed_message():
    # Create a message box to inform the user about the landing status
    msg = QMessageBox()
    msg.setIcon(QMessageBox.about)  # Set the icon for the message box
    msg.setText("Drone landing successful!")  # Set the message text
    msg.setWindowTitle("Landing Status")  # Set the title of the message box
    msg.exec_()  # Display the message box

# Function to show a loading message while the GUI is initializing
def show_loading_message(timeout_ms=3000):
    # Create a loading message box that disappears after a timeout
    msg = QMessageBox()
    colored_message = "<font color='blue'>Loading drone feed </font>"  # Set colored loading message
    msg.setText(colored_message)  # Set the message text
    msg.setWindowTitle("Initialization")  # Set the title of the message box
    msg.setStandardButtons(QMessageBox.Ok)  # Add an OK button

    timer = QTimer()  # Create a timer for the message box
    timer.setSingleShot(True)  # Ensure the timer only fires once
    timer.timeout.connect(msg.close)  # Connect the timeout event to close the message box
    timer.start(timeout_ms)  # Start the timer with the specified timeout

    msg.exec_()  # Display the message box

# GUI Class
class TelloGUI(QMainWindow):
    def __init__(self):
        super().__init__() # Initialize the parent class
        self.setWindowTitle("Drone Controller")
        self.setGeometry(500, 250, 850, 650)

        # Set focus policy for keyboard events
        self.setFocusPolicy(Qt.StrongFocus)
        self.set_background("Images/Gold-Brayer2.png")

        # Connect to the drone and start video streaming
        drone.connect()
        drone.streamon()

        # Make sure we are using the forward facing camera
        drone.set_video_direction(drone.CAMERA_FORWARD)

        # Initialize tracking and camera state variables
        self.track_marker = False
        self.find_marker = False
        self.camera_down = False
        self.human_tracking = False

        # Set up timers for updating the video frame, battery status, and temperature
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(10) # Start the timer with a 10 ms interval

        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery)
        self.battery_timer.start(10) # Start the timer with a 10 ms interval

        self.temp_timer = QTimer()
        self.temp_timer.timeout.connect(self.update_temp)
        self.temp_timer.start(10) # Start the timer with a 10 ms interval

        # Initialize ArUco dictionary and parameters
        
        self.initUI() # Initialize the user interface
        show_loading_message() # Show a loading message while the user interface is initializing

    def initUI(self):
        # Central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()  # Create a vertical layout for the main window
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
        self.switch_camera_button.clicked.connect(toggle_camera)

        switch_button_layout = QHBoxLayout() # Create a horizontal layout for the switch button
        switch_button_layout.setAlignment(Qt.AlignCenter)
        switch_button_layout.addWidget(self.switch_camera_button)

        # --- Video Feed Placeholder ---
        self.video_label = QLabel(self) # Create a label for displaying video feed
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 16px;")
        self.video_label.setAlignment(Qt.AlignCenter)

        # --- Spacer ---
        spacer = QLabel("")
        spacer.setFixedHeight(10)

        # Add widgets to the main layout
        main_layout.addWidget(self.battery_label)
        main_layout.addWidget(self.temp_label)
        main_layout.addLayout(switch_button_layout)
        main_layout.addWidget(self.video_label)
        main_layout.addWidget(spacer)

        # --- Control Buttons (Horizontally below video) ---
        button_layout = QHBoxLayout() # Create a horizontal layout for control buttons
        button_layout.setAlignment(Qt.AlignCenter)

        # List of button details: (button text, click function, style)
        button_details = [
            ("Take Off", self.takeoff, "background-color: green;"),
            ("Land", self.land, "background-color: #f1c40f;"),
            ("Go To ArUco", self.start_landing_thread, "background-color: #3498db;"),
            ("Emergency Shut Off", drone.emergency, "background-color: red;"),
            ("Toggle ArUco", toggle_aruco_dict, "background-color: #34495e;"),
            ("Toggle Human", self.toggle_human_tracking, "background-color: #e67e22;")
            # ("Track Face", self.track_face, "background-color: #e67e22;"),  # Uncomment function
            # ("Track Body", self.track_body, "background-color: #2980b9;")   # Uncomment function
        ]

        # Create buttons for drone control and tracking
        for text, function, style in button_details:
            button = QPushButton(text)
            button.clicked.connect(function)  # Connect to the appropriate function
            button.setStyleSheet(f"font-size: 16px; font-weight: bold; color: white; {style}")
            button.setFixedSize(160, 40)
            button_layout.addWidget(button)

        # Add the button layout to the main layout
        main_layout.addLayout(button_layout)

        container = QWidget() # Create a container widget
        container.setLayout(main_layout)  # Set the main layout for the container
        self.setCentralWidget(container)  # Set the container as the central widget

    # def toggle_aruco_dict(self):
    #     global aruco_dict_type
    #     if aruco_dict_type == aruco.DICT_6X6_50:
    #         aruco_dict_type = aruco.DICT_4X4_50
    #         print("Switched to ArUco Dictionary: 4x4_50")
    #     else:
    #         aruco_dict_type = aruco.DICT_6X6_50
    #         print("Switched to ArUco Dictionary: 6x6_50")

    def set_background(self, image_path):
        # Set the background of the main window using an image
        palette = QPalette()
        pixmap = QPixmap(image_path)
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.setPalette(palette)

    def update_frame(self):
        # Update the video frame displayed in the GUI
        frame_read = drone.get_frame_read()
        if frame_read is None:
            print("Failed to get frame read object.")  # Log error if frame read fails
            return
        
        img = frame_read.frame # Extract the frame
        if img is None or img.size == 0:
            print("Warning: Empty frame received from drone. Skipping...")
            return

        # Resize and process the image for ArUco detection
        img = cv2.resize(img, (640, 480))
        aruco_info, area, processed_frame = find_aruco(img)  # Detect ArUco markers

        # Convert to QImage and display
        h, w, ch = processed_frame.shape  # Get dimensions of the processed frame
        bytes_per_line = ch * w  # Calculate bytes per line for QImage
        qt_image = QImage(processed_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)  # Create QImage
        self.video_label.setPixmap(QPixmap.fromImage(qt_image))  # Set the pixmap for the video label

    def update_battery(self):
        try:
            # Update the battery status label
            battery = drone.get_battery()
            self.battery_label.setText(f"Battery: {battery}%")
            if battery < 20:
                self.battery_label.setStyleSheet("color: red;")  # Change color to red if battery is low
            else:
                self.battery_label.setStyleSheet("color: white;")  # Reset color to white if battery is okay
        except Exception as e:
            print(f"Error getting battery status: {e}")
            drone.end()
            cv2.destroyAllWindows()
            self.battery_label.setText("Battery: N/A")  # Set to N/A if error occurs
            self.battery_label.setStyleSheet("color: white;")  # Reset color to white if battery is okay

    def update_temp(self):
        # Update the temperature status label
        try:
            temp = drone.get_temperature()
            self.temp_label.setText(f"Temperature: {temp}°C")
            if temp > 85:
                self.temp_label.setStyleSheet("color: red;")  # Change color to red if temperature is high
            else:
                self.temp_label.setStyleSheet("color: white;")  # Reset color to white if temperature is okay
        except Exception as e:
            print(f"Error getting temperature status: {e}")
            drone.end()
            cv2.destroyAllWindows()
            self.temp_label.setText("Temperature: N/A")  # Set to N/A if error occurs
            self.temp_label.setStyleSheet("color: white;")  # Reset color to white if temperature is okay

    def takeoff(self):
        # Command the drone to take off
        try:
            drone.takeoff()
            # time.sleep(5)  # Optional: Wait for takeoff to stabilize
        except Exception as e:
            print(f"Error during takeoff: {e}") # Log any errors encountered during takeoff

    def land(self):
        # Command the drone to land
        try:
            drone.land()
            show_landed_message() # Show a message indicating successful landing
        except Exception as e:
            print(f"Error during landing: {e}") # Log any errors encountered during landing

    def start_landing_thread(self):
        # Start a new thread for the landing sequence
        self.thread = threading.Thread(target=self.safe_go_to_aruco_and_land, daemon=True)
        self.thread.start()

    def start_human_tracking(self):
        self.thread = threading.Thread(target=self.track_human, daemon=True)
        self.thread.start()

    def toggle_human_tracking(self):
        if not self.human_tracking:
            print("Starting human tracking...")
            self.human_tracking = True
            self.thread = threading.Thread(target=self.track_human, daemon=True)
            self.thread.start()
        else:
            print("Stopping human tracking...")
            self.human_tracking = False
    
    def track_human(self):
        global pError
        while self.human_tracking:
            try:
                img = drone.get_frame_read().frame
                if img is None or img.size == 0:
                    continue

                frame = cv2.resize(img, (640, 480))
                info, annotated_frame = find_human(frame, tracking_enabled=self.human_tracking)

                if info[0] != 0:
                    error = info[0] - frame.shape[1] // 2
                    p = PID[0] * error
                    i = PID[1] * (error + pError)
                    d = PID[2] * (error - pError)
                    yaw_velocity = int(p + i + d)
                    yaw_velocity = max(min(yaw_velocity, SPEED), -SPEED)

                    drone.send_rc_control(0, 0, 0, yaw_velocity)
                    print(f"Yaw to track human: {yaw_velocity}")

                    pError = error
                else:
                    drone.send_rc_control(0, 0, 0, 0)

                time.sleep(0.1)

            except Exception as e:
                print(f"Error in human tracking: {e}")
                break

        # After loop exits
        drone.send_rc_control(0, 0, 0, 0)
        print("Exited human tracking loop.")
        self.human_tracking = False

    def safe_go_to_aruco_and_land(self):
        # Execute the landing sequence safely
        try:
            print("Starting ArUco landing sequence...")
            go_to_aruco_and_land() # Call the function to go to the ArUco marker and land
        except Exception as e:
            print(f"Landing failed: {e}") # Log any errors encountered during landing
        finally:
            try:
                # drone.send_rc_control(0, 0, 0, 0)  # Optional: Stop all controls
                drone.end()  # End the drone session
            except Exception as e:
                print(f"[WARN] Cleanup error: {e}") # Log any cleanup errors

    def closeEvent(self, event):
        # Handle the close event for the main window
        drone.streamoff()  # Stop the video stream
        drone.end()  # Disconnect from the drone
        event.accept()  # Accept the close event

    def keyPressEvent(self, event):
        # Handle key press events for manual control
        key = event.key()  # Get the pressed key
        
        # Control drone movement based on key presses
        if key == Qt.Key_Escape:
            self.close()
        elif key == Qt.Key_W:
            drone.send_rc_control(0, 30, 0, 0)  # Forward
            time.sleep(0.5)
        elif key == Qt.Key_S:
            drone.send_rc_control(0, -30, 0, 0)  # Backward
            time.sleep(0.5)
        elif key == Qt.Key_A:
            drone.send_rc_control(-30, 0, 0, 0)  # Left
            time.sleep(0.5)
        elif key == Qt.Key_D:
            drone.send_rc_control(30, 0, 0, 0)  # Right
            time.sleep(0.5)
        elif key == Qt.Key_Up:
            drone.send_rc_control(0, 0, 30, 0)  # Up
            time.sleep(0.5)
        elif key == Qt.Key_Down:
            drone.send_rc_control(0, 0, -30, 0)  # Down
            time.sleep(0.5)
        elif key == Qt.Key_E:
            drone.send_rc_control(0, 0, 0, 30) # Rotate clockwise
            time.sleep(0.5)
        elif key == Qt.Key_Q:
            drone.send_rc_control(0, 0, 0, -30)  # Rotate counter-clockwise
            time.sleep(0.5)
        elif key == Qt.Key_C:
            toggle_camera()  # Camera toggle on 'C' key

    def keyReleaseEvent(self, event):
        # Stop all drone movement when a key is released
        drone.send_rc_control(0, 0, 0, 0)  # Stop all movement when key is released

if __name__ == '__main__':
    app = QApplication(sys.argv) # Create the application
    gui = TelloGUI()  # Create an instance of the GUI
    gui.show()
    sys.exit(app.exec_())  # Start the application event loop