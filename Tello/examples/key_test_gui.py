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
import matplotlib.pyplot as plt

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
hazards = []

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
                forward_speed = max(5 ,int(SPEED * (1 - min(area / (LANDING_THRESHOLD * 1.5), 1))))
                
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

    def update_position(self, right_cm=0, forward_cm=0, up_cm=0, yaw_change_deg=0):
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
    def __init__(self, p=0.3, i=0.0005, d=0.1):
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
my_pid_controller = PIDController(p=0.3, i=0.0005, d=0.1)

def dynamic_pid_tuning(p_gain, i_gain, d_gain):
    my_pid_controller.update(p_gain=p_gain, i_gain=i_gain, d_gain=d_gain)
    print(f"PID updated: P={p_gain}, I={i_gain}, D={d_gain}")

def circle_around_target(radius_cm=100, yaw_rate_deg_s=20):
    print("Starting circle around target...")

    # Calculate the circumference of the circle
    circumference = 2 * math.pi * radius_cm  # in cm

    # How long should it take to complete the circle?
    circle_time = circumference / SPEED  # in seconds

    # How much yaw to apply to complete 360° in that time
    yaw_speed = int(360 / circle_time)  # degrees per second

    start_time = time.time()

    while time.time() - start_time < circle_time:
        drone.send_rc_control(0, SPEED, 0, yaw_speed)  # forward + yaw
        time.sleep(0.05)  # 20Hz control loop

    drone.send_rc_control(0, 0, 0, 0)  # Stop
    print("Finished circling.")

def return_to_start():
    print("Returning to starting point...")
    current_x, current_y, current_z, current_yaw = tracker.get_position()

    # Define a helper to only move if distance is significant
    def safe_move(func, distance):
        if abs(distance) >= 20:  # Tello requires minimum ~20cm move
            func

    safe_move(drone.move_back(current_x), current_x)
    tracker.update_position(forward_cm=-current_x)

    safe_move(drone.move_left(current_y), current_y)
    tracker.update_position(right_cm=-current_y)

    safe_move(drone.move_down(current_z), current_z)
    tracker.update_position(up_cm=-current_z)

def auto_hover():
    print("Auto hover engaged.")
    drone.send_rc_control(0, 0, 0, 0)

def hazard_detected(x, y, z, yaw):
    x, y, z, yaw = tracker.get_position()
    print(f"Hazard logged at (x={x}cm, y={y}cm, z={z}cm, yaw={yaw}°)")
    
    # Optionally save to file or cloud
    with open("hazard_log.txt", "a") as f:
        f.write(f"Hazard at (x={x}cm, y={y}cm, z={z}cm, yaw={yaw}°)\n")
    hazards.append((x, y))

def grid_search_mode_rc(area_width_cm=300, area_height_cm=300, lane_spacing_cm=50):
    print("Starting grid search with RC...")
    time_per_lane = area_width_cm / SPEED
    rows = int(area_height_cm / lane_spacing_cm)
    
    for row in range(rows):
        # Move across
        if row % 2 == 0:
            print(f"Row {row + 1}: Moving right {area_width_cm} cm")
            drone.send_rc_control(SPEED, 0, 0, 0)
            tracker.update_position(SPEED)
        else:
            print(f"Row {row + 1}: Moving left {area_width_cm} cm")
            drone.send_rc_control(-SPEED, 0, 0, 0)
            tracker.update_position(-SPEED)
        
        # Continuously check for hazards during the movement
        x,y,z,yaw=tracker.get_position()  # Get position while moving
        if is_proximity_hazard(z):  # Check if the drone is too close to an obstacle
            hazard_detected(x, y, z, yaw)  # Log hazard if detected

        # Optionally capture the camera feed and check for obstacles visually
        camera_frame = drone.get_frame_read().frame  # Assuming this gives the camera feed
        if is_proximity_hazard(z, camera_frame):  # Check visually for obstacles
            hazard_detected(x, y, z, yaw)  # Log hazard if detected

        time.sleep(time_per_lane)
        drone.send_rc_control(0, 0, 0, 0)  # Stop movement

        # Move forward between lanes
        if row != rows - 1:
            print(f"Row {row + 1}: Moving forward {lane_spacing_cm} cm")
            drone.send_rc_control(0, SPEED, 0, 0)
            tracker.update_position(SPEED)
            time.sleep(lane_spacing_cm / SPEED)
            drone.send_rc_control(0, 0, 0, 0)  # Stop movement

    # After the grid search is completed, plot the hazards
    plot_hazards()

def is_proximity_hazard(z, camera_frame=None):
    # Hazard detection based on altitude (z < 50 cm) as an example
    if z < 50:  # Threshold for proximity (e.g., too low to the ground)
        return True
    
    # Optionally add camera-based obstacle detection (e.g., using computer vision)
    if camera_frame is not None:
        # Process the frame to check for obstacles
        # Example: simple check for a bright object as an obstacle (not a robust solution)
        gray_frame = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray_frame, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If any contours are detected, we assume there's an obstacle
        if len(contours) > 0:
            return True
    
    return False

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

def plot_hazards():
    if hazards:
        x_vals, y_vals = zip(*hazards)  # Unzip list of (x, y) coordinates
        plt.figure(figsize=(8, 6))
        plt.scatter(x_vals, y_vals, color='red', marker='x')
        plt.title("Hazard Locations")
        plt.xlabel("X (cm)")
        plt.ylabel("Y (cm)")
        
        # Save the plot to a file
        plt.savefig('hazards_map.png')  # Save as a PNG file
        plt.close()  # Close the plot to avoid window popping up
    else:
        print("No hazards to plot.")

class SmoothFilter:
    def __init__(self, window_size=5):
        self.values = deque(maxlen=window_size)

    def update(self, new_value):
        self.values.append(new_value)
        return sum(self.values) / len(self.values)
smooth_x = SmoothFilter()
smooth_y = SmoothFilter()

def detect_face_offset(frame):
    """Detect face and return x and y offset from center."""
    (face_center, _) = find_human(frame)  # Unpack correctly

    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2
    face_center_x, face_center_y = face_center

    offset_x = face_center_x - frame_center_x
    offset_y = frame_center_y - face_center_y  # Positive = up

    return offset_x, offset_y, frame

def smooth_follow(setpoint_x=0, setpoint_y=0):
    pid_x = PIDController(p=0.3, i=0.0005, d=0.1)
    pid_y = PIDController(p=0.3, i=0.0005, d=0.1)

    frame = drone.get_frame_read().frame  # Get the latest frame

    if frame is None:
        print("[WARN] No frame received.")
        return

    offset_x, offset_y, _ = detect_face_offset(frame)

    filtered_x = smooth_x.update(offset_x)
    filtered_y = smooth_y.update(offset_y)

    x_output = pid_x.compute(setpoint_x, filtered_x)
    y_output = pid_y.compute(setpoint_y, filtered_y)

    drone.send_rc_control(int(x_output), 0, int(y_output), 0)

# Assume you already have a working drone video feed frame
def get_face_offset_x(frame):
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
        self.setGeometry(500, 250, 950, 700)

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

        main_layout = QHBoxLayout()  # Create a vertical layout for the main window
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
        self.battery_label = QLabel(f'{battery}')
        temp = self.update_temp
        self.temp_label = QLabel(f'{temp}')

        for label in [self.battery_label, self.temp_label]:
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            label.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
            top_bar_layout.addWidget(label)

        # Create a button for the gear icon
        self.gear_button = QPushButton() # NEW
        self.gear_button.setIcon(QIcon("./Images/gear.png"))# NEW
        self.gear_button.setIconSize(QSize(32, 32))# NEW
        self.gear_button.setFixedSize(40, 40)# NEW
        self.gear_button.setStyleSheet("background-color: transparent; border: none;")# NEW
        self.gear_button.clicked.connect(self.animate_side_menu)# NEW

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
            ("Grid Search Mode", grid_search_mode_rc, "#27ae60"),
            ("Hazard Detected", hazard_detected, "#e84393"),
        ]

        for text, function, color in feature_buttons:
            button = QPushButton(text)
            button.clicked.connect(function)
            button.setStyleSheet(f"font-size: 16px; font-weight: bold; color: white; background-color: {color};")
            button.setFixedSize(180, 40)
            button.clicked.connect(lambda _, f=function: threading.Thread(target=f, daemon=True).start())
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

        # --- Speed Control Title ---
        self.speed_title_label = QLabel("Drone Speed")
        self.speed_title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: black;")
        self.side_menu_layout.addWidget(self.speed_title_label)

        # Layout for speed slider and value label
        speed_layout = QHBoxLayout()

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(60)
        self.speed_slider.setTickInterval(5)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.valueChanged.connect(self.update_speed)
        self.speed_slider.setStyleSheet("height: 20px;")

        self.speed_value_label = QLabel(f"{self.speed_slider.value()}")  # New label showing current value
        self.speed_value_label.setFixedWidth(40)  # Small width
        self.speed_value_label.setStyleSheet("font-size: 14px; font-weight: bold; color: black;")

        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_value_label)

        self.side_menu_layout.addLayout(speed_layout)

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
    
    def update_speed(self):
        global SPEED
        SPEED = self.speed_slider.value()
        self.speed_value_label.setText(str(SPEED))  # Update the label live
        print(f"Speed updated to {SPEED}")

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
                self.battery_label.setStyleSheet("color: red; font-size: 20px; font-weight: bold;")  # Change color to red if battery is low
            else:
                self.battery_label.setStyleSheet("color: black; font-size: 20px; font-weight: bold;")  # Reset color to white if battery is okay
        except Exception as e:
            print(f"Error getting battery status: {e}")
            self.battery_label.setText("Battery: N/A")  # Set to N/A if error occurs
            self.battery_label.setStyleSheet("color: black; font-size: 20px; font-weight: bold;")  # Reset color to white if battery is okay
            drone.end()
            cv2.destroyAllWindows()

    def update_temp(self):
        # Update the temperature status label
        try:
            temp = drone.get_temperature()
            self.temp_label.setText(f"Temperature: {temp}°C")
            if temp >= 100:
                self.temp_label.setStyleSheet("color: red; font-size: 20px; font-weight: bold;")  # Change color to red if temperature is really high
            elif temp >= 90 and temp < 100:
                self.temp_label.setStyleSheet("color: orange; font-size: 20px; font-weight: bold;") # Change color to orange if temperature is high
            elif temp >= 80 and temp < 90:
                self.temp_label.setStyleSheet("color: yellow; font-size: 20px; font-weight: bold;") # Change color to red if temperature is slightly high
            else:
                self.temp_label.setStyleSheet("color: black; font-size: 20px; font-weight: bold;")
        except Exception as e:
            print(f"Error getting temperature status: {e}")
            self.temp_label.setText("Temperature: N/A")  # Set to N/A if error occurs
            self.temp_label.setStyleSheet("color: black; font-size: 20px; font-weight: bold;")  # Reset color to white if temperature is okay
            drone.end()
            cv2.destroyAllWindows()

    def takeoff(self):
        # Command the drone to take off
        try:
            drone.takeoff()
            tracker.update_position(0,0,0,0)
        except Exception as e:
            print(f"Error during takeoff: {e}") # Log any errors encountered during takeoff

    def land(self):
        # Command the drone to land
        try:
            drone.land() # Show a message indicating successful landing
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
            self.move_drone(fb=30)  # Forward
        elif key == Qt.Key_S:
            self.move_drone(fb=-30)  # Backward
        elif key == Qt.Key_A:
            self.move_drone(lr=-30)  # Left
        elif key == Qt.Key_D:
            self.move_drone(lr=30)  # Right
        elif key == Qt.Key_Up:
            self.move_drone(ud=30)  # Up
        elif key == Qt.Key_Down:
            self.move_drone(ud=-30)  # Down
        elif key == Qt.Key_E:
            self.move_drone(yaw=30)  # Rotate clockwise
        elif key == Qt.Key_Q:
            self.move_drone(yaw=-30)  # Rotate counterclockwise
        elif key == Qt.Key_C:
            toggle_camera()

    def move_drone(self, lr=0, fb=0, ud=0, yaw=0):
        drone.send_rc_control(lr, fb, ud, yaw)
        tracker.update_position(lr, fb, ud, yaw)

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
