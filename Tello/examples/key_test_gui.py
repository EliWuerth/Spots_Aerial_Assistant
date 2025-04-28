import sys
import threading
import cv2
import cv2.aruco as aruco
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QMessageBox, QSizePolicy, QSlider, QFrame, QComboBox, QCheckBox)
from PyQt5.QtGui import QPalette, QPixmap, QBrush, QImage, QIcon
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, QRect, QSize
from djitellopy import Tello
import time
import math
from collections import deque

# ArUco tracking setup
drone = Tello()
aruco_tracking = False
pError = 0
camera_down = False
face_tracking = False
body_tracking = False
human_tracking = False
aruco_dict_type = aruco.DICT_6X6_50
aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
aruco_params = aruco.DetectorParameters()
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
profile_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_profileface.xml')
full_body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')
upper_body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_upperbody.xml')
lower_body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_lowerbody.xml')
smoothed_cx, smoothed_cy = None, None
alpha = 0.7
last_aruco_info = [0, 0]
last_aruco_area = 0
last_aruco_image = None
some_log = []

# Constants
SPEED = 60
PID = [0.3, 0.0005, 0.1]  # Reduced proportional gain
LANDING_THRESHOLD = 1500  # Increased to avoid premature landing
DISTANCE_THRESHOLD = 0  # Distance threshold for moving forward
SWITCH_UP_THRESHOLD = 22
SWITCH_DOWN_THRESHOLD = 22

# Function to find a human face in the provided image frame
def find_human(img):
    global face_tracking

    # Return early if there's no image or face tracking is disabled
    if img is None or not face_tracking:
        return [0, 0], img

    # Convert the image to grayscale to improve face detection performance
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Try each classifier in order of preference
    detectors = [('Face', face_cascade), ('Profile', profile_cascade), ('Full Body', full_body_cascade), ('Upper Body', upper_body_cascade), ('Lower Body', lower_body_cascade),]

    for label, cascade in detectors:
        bodies = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        if len(bodies) > 0:
            (x, y, w, h) = bodies[0]  # Use the first detected target
            cx = x + w // 2
            cy = y + h // 2

            # Draw bounding box and label
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
            
            # Return the center coordinates of the face and the annotated image
            return [cx, cy], img

    # If no face is found, return default coordinates and the original image
    return [0, 0], img

# Function to track a detected human face using the drone
def track_human():
    global pError, face_tracking
    face_tracking = True

    while face_tracking:
        try:
            # Get the current frame from the drone's camera
            img = drone.get_frame_read().frame

            # If the frame is invalid or empty, skip this iteration
            if img is None or img.size == 0:
                continue

            frame = cv2.resize(img, (640, 480))

            # Detect face and get center coordinates from the frame
            info, annotated_frame = find_human(frame)

            if info[0] != 0:
                # Calculate the error between face center and frame center (x-axis)
                error = info[0] - frame.shape[1] // 2

                # Apply PID controller to compute yaw adjustment
                p = PID[0] * error
                i = PID[1] * (error + pError)
                d = PID[2] * (error - pError)
                yaw_velocity = int(p + i + d)

                # Constrain yaw velocity within allowable speed limits
                yaw_velocity = max(min(yaw_velocity, SPEED), -SPEED)

                # Send control command to adjust drone's yaw (rotation)
                drone.send_rc_control(0, 0, 0, yaw_velocity)

                print(f"Yaw to track human: {yaw_velocity}")

                # Update previous error for PID calculation
                pError = error
            else:
                # If no face is detected, stop yaw movement
                drone.send_rc_control(0, 0, 0, 0)

            time.sleep(0.1)
        except Exception as e:
            # Print any error that occurs and break the loop
            print(f"Error in human tracking: {e}")
            break

    face_tracking = False

# Function to find aruco marker
def find_aruco(img):
    global aruco_tracking, last_aruco_info, last_aruco_area, last_aruco_image

    if img is None or img.size == 0:
        print("Warning: Received an empty frame in find_aruco.")
        last_aruco_info, last_aruco_area, last_aruco_image = [0, 0], 0, np.zeros((480, 640, 3), dtype=np.uint8)
        return last_aruco_info, last_aruco_area, last_aruco_image

    if not aruco_tracking:
        last_aruco_info, last_aruco_area, last_aruco_image = [0, 0], 0, img
        return last_aruco_info, last_aruco_area, last_aruco_image

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    current_dict = aruco.getPredefinedDictionary(aruco_dict_type)
    corners, ids, _ = aruco.detectMarkers(gray, current_dict, parameters=aruco_params)

    if ids is not None and len(ids) > 0:
        aruco.drawDetectedMarkers(img, corners, ids)
        c = corners[0][0]
        cx = int(np.mean(c[:, 0]))
        cy = int(np.mean(c[:, 1]))
        area = cv2.contourArea(c.astype(np.float32))
        cv2.circle(img, (cx, cy), 5, (0, 255, 255), cv2.FILLED)

        last_aruco_info, last_aruco_area, last_aruco_image = [cx, cy], area, img
        return last_aruco_info, last_aruco_area, last_aruco_image
    else:
        last_aruco_info, last_aruco_area, last_aruco_image = [0, 0], 0, img
        return last_aruco_info, last_aruco_area, last_aruco_image

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
    global aruco_tracking, pError, camera_down, aruco_dict_type
    aruco_tracking = True  # Enable ArUco tracking
    last_seen_time = time.time()  # Record the last time the marker was seen
    landed = False  # Flag to indicate if the drone has landed
    marker_lost = False  # Flag to indicate if the marker has been lost
    zero_speed_start_time = None # Store the last time forward_speed was 0

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
                """ show_landed_message() """  # Show landing success message
            elif aruco_info[0] != 0 and (camera_down == False):
                # print("Marker Found")  # Marker detected
                last_seen_time = time.time()  # Update last seen time
                cx, cy = aruco_info  # Get the center coordinates of the marker
                error_x = cx - frame.shape[1] // 2  # Calculate horizontal error from the center
                error_y = cy - frame.shape[0] // 2  # Calculate vertical error from the center

                # PID calculations
                yaw_velocity, pError = track_aruco(aruco_info, frame.shape[1], PID, pError)
                
                # Forward control
                forward_speed = int(SPEED * (1 - min(area / (LANDING_THRESHOLD * 1.5), 1)))
                
                # Vertical adjustment based on vertical position
                vertical_speed = int(PID[0] * error_y)
                
                # Only move forward if marker is roughly centered (error within threshold)
                if forward_speed > DISTANCE_THRESHOLD:
                    drone.send_rc_control(0, forward_speed, -vertical_speed, yaw_velocity)  # Move the drone
                    tracker.update_position()
                    time.sleep(0.1)  # Wait for a second before the next control command
                    if aruco_dict_type == aruco.DICT_6X6_50 and forward_speed <= SWITCH_DOWN_THRESHOLD:
                        aruco_dict_type = aruco.DICT_4X4_50
                    elif aruco_dict_type == aruco.DICT_4X4_50 and forward_speed >= SWITCH_UP_THRESHOLD:
                        aruco_dict_type = aruco.DICT_4X4_50
                elif forward_speed < DISTANCE_THRESHOLD:
                    toggle_camera()  # Switch camera view
                    bottom_camera_and_land()  # Attempt to land using the bottom camera  # Maintain position
                    
                print(f"Area: {area}, Forward: {forward_speed}, Vertical: {vertical_speed}, Yaw: {yaw_velocity}, error: {error_x}") # Log control values
                
            elif aruco_info[0] == 0 :
                # Handle the case where the marker is lost
                if time.time() - last_seen_time > 2 and not marker_lost:  # Check if marker has been lost for more than 2 seconds
                    print(f"Area: {area}, Forward: {forward_speed}, Vertical: {vertical_speed}, Yaw: {yaw_velocity}") # Log control values
                    marker_lost = True  # Prevent re-execution of lost marker logic
                    print("Marker lost - hovering")  # Indicate that the marker is lost

                    drone.send_rc_control(0, 0, 0, 0)  # Hover in place
                    time.sleep(0.05)  # Wait for a second

                    toggle_camera()  # Switch camera view

                    # Drone hovers while searching for the marker
                    drone.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.05)  # Wait before attempting to land again
                    
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
    
    # Get the frame from the drone's bottom camera
    frame = drone.get_frame_read().frame
    
    # Resize the frame for processing
    frame = cv2.resize(frame, (640, 480))
    aruco_info, area, processed_frame = find_aruco(frame)  # Find ArUco markers in the frame
    
    if aruco_info[0] == 0 and not landed:
        # If the marker is lost, start searching for it
        print("Marker not found, starting search...")
        drone.send_rc_control(0,0,0,0) # Hover in place
        time.sleep(1)  # Wait for a second before searching again

    elif aruco_info[0] != 0 and not landed:
        # Command the drone to land if the marker is found
        drone.land()
        landed = True  # Set the landed flag to True
        # show_landed_message()  # Show landing success message

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

# Function to toggle between two types of ArUco marker dictionaries
def toggle_aruco_dict():
    global aruco_dict_type
    
    # Toggle between two ArUco dictionaries
    if aruco_dict_type == aruco.DICT_6X6_50:
        aruco_dict_type = aruco.DICT_4X4_50
        print("Switched to ArUco Dictionary: 4x4_50")
    else:
        aruco_dict_type = aruco.DICT_6X6_50
        print("Switched to ArUco Dictionary: 6x6_50")

    return aruco_dict_type

class PositionTracker:
    def __init__(self):
        self.x = 0  # cm
        self.y = 0  # cm
        self.z = 0  # cm
        self.yaw = 0  # degrees
        self.last_update_time = time.time()

    def reset(self):
        self.x = self.y = self.z = 0
        self.yaw = 0
        self.last_update_time = time.time()

    def update_position(self, forward_cm=0, right_cm=0, up_cm=0, yaw_change_deg=0):
        """
        Update position estimate based on commanded movements.
        """
        # Update yaw
        self.yaw += yaw_change_deg
        self.yaw %= 360

        # Convert movement relative to yaw angle
        yaw_rad = math.radians(self.yaw)

        dx = forward_cm * math.cos(yaw_rad) + right_cm * math.cos(yaw_rad + math.pi/2)
        dy = forward_cm * math.sin(yaw_rad) + right_cm * math.sin(yaw_rad + math.pi/2)

        self.x += dx
        self.y += dy
        self.z += up_cm

        self.last_update_time = time.time()

    def get_position(self):
        return (int(self.x), int(self.y), int(self.z), int(self.yaw))
tracker = PositionTracker()

class PIDController:
    def __init__(self, p=0.1, i=0.0, d=0.0):
        self.kp = p
        self.ki = i
        self.kd = d

        self.previous_error = 0
        self.integral = 0

    def update(self, p_gain=None, i_gain=None, d_gain=None):
        if p_gain is not None:
            self.kp = p_gain
        if i_gain is not None:
            self.ki = i_gain
        if d_gain is not None:
            self.kd = d_gain

    def compute(self, setpoint, measured_value):
        """
        Calculate PID output
        """
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.previous_error = error
        return output
my_pid_controller = PIDController(p=0.1, i=0.0, d=0.0)

def dynamic_pid_tuning(p_gain, i_gain, d_gain):
    my_pid_controller.update(p_gain=p_gain, i_gain=i_gain, d_gain=d_gain)
    print(f"PID updated: P={p_gain}, I={i_gain}, D={d_gain}")

def circle_around_target(radius_cm=100, speed=30):
    print("Starting circle around target...")
    # Simple circle — fake by stepping yaw while moving forward a bit
    for _ in range(36):  # 10 degrees each time, full 360
        drone.move_forward(int((2 * math.pi * radius_cm) / 36))
        drone.rotate_clockwise(10)
        tracker.update_position(forward_cm=(2 * math.pi * radius_cm) / 36, yaw_change_deg=10)

def return_to_start():
    print("Returning to starting point...")
    current_x, current_y, current_z, current_yaw = tracker.get_position()
    drone.move_back(int(current_x))
    tracker.update_position(forward_cm=-current_x)
    drone.move_left(int(current_y))
    tracker.update_position(right_cm=-current_y)
    drone.move_down(int(current_z))
    tracker.update_position(up_cm=-current_z)

def auto_hover():
    print("Auto hover engaged.")
    drone.send_rc_control(0, 0, 0, 0)

def smooth_follow(setpoint_x=0, setpoint_y=0):
    """
    Imagine you have face detection or marker tracking.
    setpoint_x, setpoint_y: desired center (e.g., frame center 0,0)
    """
    current_x = get_face_offset_x()  # Write this for your camera detection
    current_y = get_face_offset_y()

    x_output = my_pid_controller.compute(setpoint_x, current_x)
    y_output = my_pid_controller.compute(setpoint_y, current_y)

    # Clamp output to max/min drone speeds
    x_output = int(max(min(x_output, 100), -100))
    y_output = int(max(min(y_output, 100), -100))

    drone.send_rc_control(x_output, 0, y_output, 0)
    tracker.update_position()

def grid_search_mode(area_width_cm=300, area_height_cm=300, lane_spacing_cm=50):
    print("Starting grid search...")
    rows = int(area_height_cm / lane_spacing_cm)
    for row in range(rows):
        # Move across
        if row % 2 == 0:
            drone.move_right(area_width_cm)
            tracker.update_position(right_cm=area_width_cm)
        else:
            drone.move_left(area_width_cm)
            tracker.update_position(right_cm=-area_width_cm)
        # Move forward between lanes
        if row != rows - 1:
            drone.move_forward(lane_spacing_cm)
            tracker.update_position(forward_cm=lane_spacing_cm)

def hazard_detected():
    x, y, z, yaw = tracker.get_position()
    print(f"Hazard logged at (x={x}cm, y={y}cm, z={z}cm, yaw={yaw}°)")
    # Optionally save to file or cloud
    with open("hazard_log.txt", "a") as f:
        f.write(f"Hazard at (x={x}cm, y={y}cm, z={z}cm, yaw={yaw}°)\n")

def get_distance_to_target(marker_size_cm=20):
    """
    Estimate distance based on marker size in image.
    """
    perceived_width_pixels = detect_marker_width()  # You have to implement
    focal_length = 700  # You may need to calibrate this

    if perceived_width_pixels == 0:
        return None  # No marker found

    distance_cm = (marker_size_cm * focal_length) / perceived_width_pixels
    return distance_cm

def grid_search_mode_rc(area_width_cm=300, area_height_cm=300, lane_spacing_cm=50, speed=30):
    print("Starting grid search with RC...")
    time_per_lane = area_width_cm / speed
    rows = int(area_height_cm / lane_spacing_cm)
    for row in range(rows):
        # Move across
        if row % 2 == 0:
            drone.send_rc_control(speed, 0, 0, 0)
            tracker.update_position()
        else:
            drone.send_rc_control(-speed, 0, 0, 0)
            tracker.update_position()
        time.sleep(time_per_lane)
        drone.send_rc_control(0, 0, 0, 0)
        # Move forward between lanes
        if row != rows - 1:
            drone.send_rc_control(0, speed, 0, 0)
            tracker.update_position()
            time.sleep(lane_spacing_cm / speed)
            drone.send_rc_control(0, 0, 0, 0)

class SmoothFilter:
    def __init__(self, window_size=5):
        self.values = deque(maxlen=window_size)

    def update(self, new_value):
        self.values.append(new_value)
        return sum(self.values) / len(self.values)
smooth_x = SmoothFilter()
smooth_y = SmoothFilter()

def smooth_follow_with_filter(setpoint_x=0, setpoint_y=0):
    current_x = get_face_offset_x()
    current_y = get_face_offset_y()

    filtered_x = smooth_x.update(current_x)
    filtered_y = smooth_y.update(current_y)

    x_output = my_pid_controller.compute(setpoint_x, filtered_x)
    y_output = my_pid_controller.compute(setpoint_y, filtered_y)

    drone.send_rc_control(int(x_output), 0, int(y_output), 0)

# Assume you already have a working drone video feed frame
def get_face_offset_x(frame):
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    frame_center_x = frame.shape[1] // 2

    if len(faces) == 0:
        return 0  # No face, assume centered

    # Use first detected face
    (x, y, w, h) = faces[0]
    face_center_x = x + w // 2

    offset_x = face_center_x - frame_center_x
    return offset_x  # Positive = right, Negative = left

def get_face_offset_y(frame):
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    frame_center_y = frame.shape[0] // 2

    if len(faces) == 0:
        return 0  # No face, assume centered

    # Use first detected face
    (x, y, w, h) = faces[0]
    face_center_y = y + h // 2

    offset_y = frame_center_y - face_center_y
    return offset_y  # Positive = up, Negative = down

# Assume you already have a working frame
def detect_marker_width(frame, target_id=0):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return 0  # No marker detected

    for idx, marker_id in enumerate(ids.flatten()):
        if marker_id == target_id:
            # corners[idx][0] are the 4 points (top-left, top-right, bottom-right, bottom-left)
            pts = corners[idx][0]
            top_left, top_right = pts[0], pts[1]
            width = int(np.linalg.norm(top_right - top_left))
            return width

    return 0  # Target marker not found

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

        self.initUI() # Initialize the user interface
        show_loading_message() # Show a loading message while the user interface is initializing

    def initUI(self):
        # Central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()  # Create a vertical layout for the main window
        central_widget.setLayout(main_layout)
        main_layout.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        # === LEFT MAIN AREA ===
        left_main_area = QVBoxLayout()#NEW
        left_main_area.setAlignment(Qt.AlignTop | Qt.AlignHCenter)#NEW

        # Top Section
        top_section = QWidget()#NEW
        top_section.setStyleSheet("background-color: white; border-radius: 10px;")#NEW
        top_bar_layout = QHBoxLayout()#NEW

        # --- Video and Temperature ---
        battery = self.update_battery
        self.battery_label = QLabel(f"{battery}")
        self.battery_label.setAlignment(Qt.AlignCenter)
        self.battery_label.setStyleSheet("font-size: 16px; color: white; font-weight: bold;")
        
        temp = self.update_temp
        self.temp_label = QLabel(f"{temp}°C")
        self.temp_label.setAlignment(Qt.AlignCenter)

        self.temp_label.setStyleSheet("font-size: 16px; color: white; font-weight: bold;")

        # Create a button for the gear icon
        self.gear_button = QPushButton() # NEW
        self.gear_button.setIcon(QIcon("gear.png"))# NEW
        self.gear_button.setIconSize(QSize(32, 32))# NEW
        self.gear_button.setFixedSize(40, 40)# NEW
        self.gear_button.setStyleSheet("background-color: transparent; border: none;")# NEW
        self.gear_button.clicked.connect(self.animate_side_menu)# NEW

        top_bar_layout.addWidget(self.battery_label) # NEW
        top_bar_layout.addWidget(self.temp_label) # NEW
        top_bar_layout.addWidget(self.gear_button)
        top_section.setLayout(top_bar_layout)

        switch_button_layout = QHBoxLayout()
        switch_button_layout.setAlignment(Qt.AlignCenter)

        # --- Switch Camera Button ---
        self.switch_camera_button = QPushButton("Switch Camera")
        self.switch_camera_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: #9b59b6; color: white;")
        self.switch_camera_button.setFixedSize(160, 40)
        self.switch_camera_button.clicked.connect(toggle_camera)

        # Small spacing between buttons
        switch_button_layout.addSpacing(20)

        # Switch ArUco Button
        self.switch_aruco_button = QPushButton("Switch ArUco")
        self.switch_aruco_button.setStyleSheet("font-size: 16px; font-weight: bold; background-color: #34495e; color: white;")
        self.switch_aruco_button.setFixedSize(160, 40)
        self.switch_aruco_button.clicked.connect(toggle_aruco_dict)

        switch_button_layout.addWidget(self.switch_camera_button)
        switch_button_layout.addWidget(self.switch_aruco_button)

        # Create a widget just to hold the button row and center it
        switch_buttons_widget = QWidget()
        switch_buttons_widget.setLayout(switch_button_layout)

        # --- Video Feed Placeholder ---
        self.video_label = QLabel() # Create a label for displaying video feed
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 16px;")
        self.video_label.setAlignment(Qt.AlignCenter)
        video_container = QHBoxLayout()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        video_container = QHBoxLayout()
        video_container.setAlignment(Qt.AlignCenter)
        video_container.addWidget(self.video_label)

        # --- Spacer ---
        spacer = QLabel("")
        spacer.setFixedHeight(10)

        # Add widgets to the main layout
        left_main_area.addWidget(top_section)
        left_main_area.addWidget(switch_buttons_widget, alignment=Qt.AlignCenter) # NEW
        left_main_area.addLayout(video_container)
        left_main_area.addWidget(spacer)

        # --- Control Buttons (Horizontally below video) ---
        button_layout = QHBoxLayout() # Create a horizontal layout for control buttons
        button_layout.setAlignment(Qt.AlignCenter)

        # List of button details: (button text, click function, style)
        button_details = [
            ("Take Off", self.takeoff, "background-color: green;"),
            ("Land", self.land, "background-color: #f1c40f;"),
            ("Go To ArUco", self.start_landing_thread, "background-color: #3498db;"),
            ("Emergency Shut Off", drone.emergency, "background-color: red;"),
            ("Find Human", self.start_human_tracking, "background-color: #e67e22;"),
        ]

        # Create buttons for drone control and tracking
        for text, function, style in button_details:
            button = QPushButton(text)
            button.clicked.connect(function)  # Connect to the appropriate function
            button.setStyleSheet(f"font-size: 16px; font-weight: bold; color: white; {style}")
            button.setFixedSize(160, 40)
            button_layout.addWidget(button)

        # --- New Features Buttons ---
        self.new_features_widget = QWidget()
        new_features_layout = QHBoxLayout()
        new_features_layout.setAlignment(Qt.AlignCenter)

        feature_buttons = [
            ("Circle Around Target", circle_around_target, "#8e44ad"),
            ("Return to Start", return_to_start, "#16a085"),
            ("Auto Hover", auto_hover, "#2980b9"),
            ("Smooth Follow", smooth_follow, "#c0392b"),
            ("Grid Search Mode", grid_search_mode, "#27ae60"),
            ("Hazard Detected", hazard_detected, "#e84393"),
        ]

        for text, function, color in feature_buttons:
            button = QPushButton(text)
            button.clicked.connect(function)
            button.setStyleSheet(f"font-size: 16px; font-weight: bold; color: white; background-color: {color};")
            button.setFixedSize(180, 40)
            new_features_layout.addWidget(button)

        self.new_features_widget.setLayout(new_features_layout)
        self.new_features_widget.setVisible(False)  # Hide initially

        left_main_area.addWidget(spacer)
        left_main_area.addLayout(button_layout)
        left_main_area.addWidget(self.new_features_widget)

        main_layout.addLayout(left_main_area)

        # === RIGHT SIDE MENU ===
        self.side_menu = QFrame()
        self.side_menu.setFixedWidth(0)  # Start hidden
        self.side_menu.setStyleSheet("background-color: rgba(255, 255, 255, 0.8); border-radius: 10px;")
        self.side_menu_layout = QVBoxLayout()
        self.side_menu_layout.setAlignment(Qt.AlignTop)

        self.pid_title_label = QLabel("PID Settings")
        self.pid_title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: black;")
        self.side_menu_layout.addWidget(self.pid_title_label)

        self.p_slider = self.create_pid_slider("P Gain", self.side_menu_layout)
        self.i_slider = self.create_pid_slider("I Gain", self.side_menu_layout)
        self.d_slider = self.create_pid_slider("D Gain", self.side_menu_layout)

        # NEW: Advanced Drone Mode Checkbox
        self.advanced_mode_checkbox = QCheckBox("Advanced Drone Mode")
        self.advanced_mode_checkbox.setStyleSheet("font-size: 14px; font-weight: bold; color: black;")
        self.advanced_mode_checkbox.stateChanged.connect(self.toggle_advanced_mode)
        self.side_menu_layout.addWidget(self.advanced_mode_checkbox)

        self.side_menu.setLayout(self.side_menu_layout)
        main_layout.addWidget(self.side_menu)

        # Create animation
        self.side_menu_animation = QPropertyAnimation(self.side_menu, b"geometry")
        self.side_menu_animation.setDuration(300)  # Duration in ms
        self.side_menu_animation.setEasingCurve(QEasingCurve.InOutCubic)

        self.side_menu_expanded = False

        container = QWidget() # Create a container widget
        container.setLayout(main_layout)  # Set the main layout for the container
        self.setCentralWidget(container)  # Set the container as the central widget
    
    def update_pid_values(self):
        p_gain = self.pid_p_slider.value() / 100.0  # Scale slider 0–10.0
        dynamic_pid_tuning(p_gain, i_gain=0.5, d_gain=0.1)  # You can have more sliders if needed

    def show_landed_message():
        # Create a message box to inform the user about the landing status
        msg = QMessageBox()
        msg.setText("Drone landing successful!")  # Set the message text
        msg.setWindowTitle("Landing Status")  # Set the title of the message box
        msg.exec_()  # Display the message box

    def set_background(self, image_path):
        # Set the background of the main window using an image
        palette = QPalette()
        pixmap = QPixmap(image_path)
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.setPalette(palette)

    def update_frame(self):
        frame_read = drone.get_frame_read()
        if frame_read is None:
            print("Failed to get frame read object.")
            return
        
        img = frame_read.frame
        if img is None or img.size == 0:
            print("Warning: Empty frame received from drone. Skipping...")
            return

        img = cv2.resize(img, (640, 480))

        if face_tracking:
            _, processed_frame = find_human(img)
        else:
            _, _, processed_frame = find_aruco(img)

        h, w, ch = processed_frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(processed_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qt_image))

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
            tracker.update_position()
            # time.sleep(5)  # Optional: Wait for takeoff to stabilize
        except Exception as e:
            print(f"Error during takeoff: {e}") # Log any errors encountered during takeoff

    def land(self):
        # Command the drone to land
        try:
            drone.land() # Show a message indicating successful landing
            tracker.update_position()
        except Exception as e:
            print(f"Error during landing: {e}") # Log any errors encountered during landing

    def start_landing_thread(self):
        # Start a new thread for the landing sequence
        self.thread = threading.Thread(target=self.safe_go_to_aruco_and_land, daemon=True)
        self.thread.start()

    def start_human_tracking(self):
        self.thread = threading.Thread(target=track_human, daemon=True)
        self.thread.start()

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
        # event.accept()  # Accept the close event

    def keyPressEvent(self, event):
        # Handle key press events for manual control
        key = event.key()  # Get the pressed key
        
        # Control drone movement based on key presses
        if key == Qt.Key_Escape:
            self.close()
        elif key == Qt.Key_W:
            drone.send_rc_control(0, 30, 0, 0)  # Forward
            tracker.update_position()
            time.sleep(0.5)
        elif key == Qt.Key_S:
            drone.send_rc_control(0, -30, 0, 0)  # Backward
            tracker.update_position()
            time.sleep(0.5)
        elif key == Qt.Key_A:
            drone.send_rc_control(-30, 0, 0, 0)  # Left
            tracker.update_position()
            time.sleep(0.5)
        elif key == Qt.Key_D:
            drone.send_rc_control(30, 0, 0, 0)  # Right
            tracker.update_position()
            time.sleep(0.5)
        elif key == Qt.Key_Up:
            drone.send_rc_control(0, 0, 30, 0)  # Up
            tracker.update_position()
            time.sleep(0.5)
        elif key == Qt.Key_Down:
            drone.send_rc_control(0, 0, -30, 0)  # Down
            tracker.update_position()
            time.sleep(0.5)
        elif key == Qt.Key_E:
            drone.send_rc_control(0, 0, 0, 30) # Rotate clockwise
            tracker.update_position()
            time.sleep(0.5)
        elif key == Qt.Key_Q:
            drone.send_rc_control(0, 0, 0, -30)  # Rotate counter-clockwise
            tracker.update_position()
            time.sleep(0.5)
        elif key == Qt.Key_C:
            toggle_camera()  # Camera toggle on 'C' key

    def keyReleaseEvent(self, event):
        # Stop all drone movement when a key is released
        drone.send_rc_control(0, 0, 0, 0)  # Stop all movement when key is released

    def create_pid_slider(self, label_text, layout):
        label = QLabel(label_text)
        label.setStyleSheet("font-size: 14px; color: black;")
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(50)
        slider.setTickInterval(1)
        slider.setStyleSheet("height: 20px;")
        layout.addWidget(label)
        layout.addWidget(slider)
        return slider

    def toggle_side_menu(self):
        if self.side_menu.isVisible():
            # Animate closing (slide left)
            start_rect = self.side_menu.geometry()
            end_rect = QRect(start_rect.x(), start_rect.y(), 0, start_rect.height())

            self.side_menu_animation.setStartValue(start_rect)
            self.side_menu_animation.setEndValue(end_rect)
            self.side_menu_animation.start()

            # After animation finishes, hide the side menu
            self.side_menu_animation.finished.connect(lambda: self.side_menu.setVisible(False))

        else:
            # First make it visible and set width 0 (collapsed)
            self.side_menu.setVisible(True)
            collapsed_rect = QRect(self.side_menu.x(), self.side_menu.y(), 0, self.side_menu.height())
            expanded_rect = QRect(self.side_menu.x(), self.side_menu.y(), 250, self.side_menu.height())  # 250px wide side menu

            self.side_menu.setGeometry(collapsed_rect)

            self.side_menu_animation.setStartValue(collapsed_rect)
            self.side_menu_animation.setEndValue(expanded_rect)
            self.side_menu_animation.start()

    def animate_side_menu(self):
        self.side_menu_animation.stop()

        if self.side_menu_expanded:
            start_width = 250
            end_width = 0
        else:
            start_width = 0
            end_width = 250

        self.side_menu_animation = QPropertyAnimation(self.side_menu, b"maximumWidth")
        self.side_menu_animation.setDuration(300)
        self.side_menu_animation.setStartValue(start_width)
        self.side_menu_animation.setEndValue(end_width)
        self.side_menu_animation.setEasingCurve(QEasingCurve.InOutCubic)
        self.side_menu_animation.start()

        self.side_menu_expanded = not self.side_menu_expanded

    def toggle_advanced_mode(self, state):
            if state == Qt.Checked:
                self.new_features_widget.setVisible(True)
            else:
                self.new_features_widget.setVisible(False)

if __name__ == '__main__':
    app = QApplication(sys.argv) # Create the application
    gui = TelloGUI()  # Create an instance of the GUI
    gui.show()
    sys.exit(app.exec_())  # Start the application event loop
