# Drone command retry wrapper
def retry_command(cmd_func, max_retries=3, timeout=2.0):
    """Execute a drone command with retry logic and timeout"""
    for attempt in range(max_retries):
        try:
            # Create an event for signaling completion
            done_event = threading.Event()
            result = [None]
            error = [None]
            
            # Function to execute in thread
            def execute():
                try:
                    result[0] = cmd_func()
                    done_event.set()
                except Exception as e:
                    error[0] = e
                    done_event.set()
            
            # Start command in a thread
            cmd_thread = threading.Thread(target=execute)
            cmd_thread.daemon = True
            cmd_thread.start()
            
            # Wait for completion or timeout
            success = done_event.wait(timeout)
            
            if success and error[0] is None:
                return result[0]  # Command succeeded
            
            if not success:
                print(f"Command timed out on attempt {attempt+1}/{max_retries}")
            else:
                print(f"Command failed on attempt {attempt+1}/{max_retries}: {error[0]}")
                
            # Wait before retry
            time.sleep(0.5)
        except Exception as e:
            print(f"Error in retry mechanism on attempt {attempt+1}/{max_retries}: {e}")
            time.sleep(0.5)
    
    print(f"Command failed after {max_retries} attempts")
    return None  # All attempts failed# Create a thread for body detection
# This will run body detection separately from the main video thread
# to prevent blocking during intensive detection operations

# Make sure threading is properly imported
try:
    import threading
    # Create a lock for thread synchronization
    body_lock = threading.Lock()
except ImportError:
    print("WARNING: Threading module not available - using fallback")
    # Create a dummy lock class as fallback
    class DummyLock:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def acquire(self, *args, **kwargs): return True
        def release(self, *args, **kwargs): pass
    body_lock = DummyLock()

body_frame = None  # Shared frame for body detection
body_result = ([0, 0], 0, None)  # Shared results

def body_detection_thread():
    global body_frame, body_result, body_tracking, body_lock
    
    while True:
        # Only process if body tracking is enabled
        if body_tracking:
            # Get the current frame with thread safety
            with body_lock:
                if body_frame is not None:
                    frame_copy = body_frame.copy()
                else:
                    time.sleep(0.1)  # Wait for a frame
                    continue
            
            # Process the frame (this won't block the main thread)
            info, area, img_result = find_body(frame_copy)
            
            # Store the result with thread safety
            with body_lock:
                body_result = (info, area, img_result)
        
        # Sleep to reduce CPU usage
        time.sleep(0.1)# Non-blocking takeoff function with timeout
def safe_takeoff():
    print("Taking off! (with timeout protection)")
    
    # Disable all tracking temporarily during takeoff
    global face_tracking, body_tracking, aruco_tracking
    prev_face_tracking = face_tracking
    prev_body_tracking = body_tracking
    prev_aruco_tracking = aruco_tracking
    
    face_tracking = False
    body_tracking = False
    aruco_tracking = False
    
    # Create a variable to track success
    takeoff_success = False
    
    # Add a timeout mechanism
    def takeoff_with_timeout():
        nonlocal takeoff_success
        try:
            # Send takeoff command
            drone.takeoff()
            takeoff_success = True
            print("Takeoff successful!")
        except Exception as e:
            print(f"Takeoff error: {e}")
            takeoff_success = False
    
    # Create a thread for takeoff
    takeoff_thread = threading.Thread(target=takeoff_with_timeout)
    takeoff_thread.daemon = True
    takeoff_thread.start()
    
    # Wait with timeout
    timeout = 7  # seconds
    start_time = time.time()
    
    # Display status to user
    print("Processing takeoff command...")
    
    # Wait for thread to complete or timeout
    while takeoff_thread.is_alive() and time.time() - start_time < timeout:
        time.sleep(0.2)  # Check status every 200ms
        print(".", end="", flush=True)  # Show progress
    
    print()  # New line
    
    # If it's still running after timeout, we have a problem
    if takeoff_thread.is_alive():
        print(f"Takeoff command timed out after {timeout} seconds!")
        print("The drone might be unresponsive - try reconnecting")
    
    # Restore previous tracking settings after 2 seconds
    time.sleep(2)  # Give the drone time to stabilize
    face_tracking = prev_face_tracking
    body_tracking = prev_body_tracking
    aruco_tracking = prev_aruco_tracking
    
    return takeoff_success# Body detection and tracking function
def find_body(img):
    global body_tracking
    
    if not body_tracking:
        return [0, 0], 0, img  # Return default values when not tracking
    
    # Detect bodies in the image
    try:
        # Make detection more efficient by using an even smaller image
        detection_img = cv2.resize(img, (160, 120))  # Much smaller for detection
        
        # Detect people using HOG descriptor with efficiency parameters
        bodies, weights = hog.detectMultiScale(
            detection_img, 
            winStride=(4, 4),     # Faster stride
            padding=(8, 8), 
            scale=1.1,           # Larger scale step for faster detection
            finalThreshold=1.5    # Higher threshold to reduce false positives
        )
        
        # Scale coordinates back to original image size
        if len(bodies) > 0:
            scaled_bodies = []
            for (x, y, w, h) in bodies:
                x2 = x * 4  # Scale back to 640x480 (160*4 = 640)
                y2 = y * 4
                w2 = w * 4
                h2 = h * 4
                scaled_bodies.append((x2, y2, w2, h2))
            bodies = scaled_bodies
        
        body_list_center = []
        body_list_area = []
        
        # Process detected bodies
        for (x, y, w, h) in bodies:
            # Draw rectangle around body
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Calculate center point
            cx = x + w // 2
            cy = y + h // 2
            area = w * h
            
            # Mark center of body
            cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
            
            body_list_center.append([cx, cy])
            body_list_area.append(area)
        
        # Find the largest body if multiple are detected
        if len(body_list_area) > 0:
            i = body_list_area.index(max(body_list_area))
            cv2.putText(img, "Body Detected", (body_list_center[i][0] - 50, body_list_center[i][1] - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            return body_list_center[i], body_list_area[i], img
    except Exception as e:
        print(f"Error in body detection: {e}")
        cv2.putText(img, "Body Detection Error", (10, 150), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Return default if no body detected or on error
    return [0, 0], 0, img

# PID controller for tracking bodies
def track_body(info, width, pid, p_error):
    global body_tracking
    
    if not body_tracking or info[0] == 0:
        return 0, p_error  # Don't track if disabled or no body
    
    # Calculate error from center
    error = info[0] - width // 2
    
    # PID calculation
    p = pid[0] * error
    i = pid[1] * (error + p_error)
    d = pid[2] * (error - p_error)
    
    # Calculate yaw velocity
    yaw_velocity = int(p + i + d)
    
    # Limit yaw velocity
    if yaw_velocity > S:
        yaw_velocity = S
    elif yaw_velocity < -S:
        yaw_velocity = -S
    
    return yaw_velocity, error# Function to attempt drone reconnection
def reconnect_drone():
    global drone, connection_successful
    print("\n--- ATTEMPTING TO RECONNECT DRONE ---")
    
    # Try to properly end the current connection first
    try:
        retry_command(
            lambda: drone.end(),
            max_retries=1,
            timeout=2.0
        )
    except:
        pass
    
    time.sleep(1)
    
    # Create a new drone object
    drone = Tello()
    
    # Try to connect again with timeout protection
    try:
        print("Connecting to Tello...")
        connection_result = retry_command(
            lambda: drone.connect(),
            max_retries=1,
            timeout=5.0
        )
        
        if connection_result is None:
            print("Connection timed out")
            connection_successful = False
            return False
        
        time.sleep(0.5)
        
        # Test connection by getting battery
        battery = retry_command(
            lambda: drone.get_battery(),
            max_retries=2,
            timeout=2.0
        )
        
        if battery is None:
            print("Could not get battery level. Connection failed.")
            connection_successful = False
            return False
            
        print(f"Reconnection successful! Battery: {battery}%")
        
        # Restart video if needed
        try:
            retry_command(
                lambda: drone.streamoff(),
                max_retries=1,
                timeout=2.0
            )
            time.sleep(0.5)
            retry_command(
                lambda: drone.streamon(),
                max_retries=1,
                timeout=2.0
            )
            print("Video stream restarted")
        except Exception as e:
            print(f"Warning: Could not restart video stream: {e}")
        
        connection_successful = True
        return True
    except Exception as e:
        print(f"Reconnection failed: {e}")
        connection_successful = False
        return False# Wrapper function for safely executing drone commands
def safe_drone_command(command_func, default_value=0):
    try:
        return command_func()
    except Exception as e:
        print(f"Drone command error: {e}")
        return default_value
# First, ensure all imports are at the top of the file
import cv2  # Make sure OpenCV is installed: pip install opencv-python
import time
import numpy as np
from djitellopy import Tello  # Make sure djitellopy is installed: pip install djitellopy
import threading
import keyboard  # Make sure keyboard is installed: pip install keyboard
import os
import cv2.aruco as aruco  # This is part of OpenCV

# Speed settings
S = 60  # Default speed
face_tracking = False  # Face tracking toggle
aruco_tracking = False  # Aruco marker tracking toggle
body_tracking = False  # Body tracking toggle
PID = [0.5, 0.0005, 0.1]  # PID control coefficients for tracking
pError = 0  # Previous error for PID controller

# Load face detection model
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Load HOG descriptor for body detection
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Set up Aruco marker detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
aruco_params = aruco.DetectorParameters()

# Initialize Tello drone with connection checking
drone = Tello()
connection_successful = False

# Try to connect with retry
for attempt in range(3):
    try:
        print(f"Connecting to Tello (attempt {attempt+1}/3)...")
        # Use timeout for connection
        connection_result = retry_command(
            lambda: drone.connect(),
            max_retries=1,
            timeout=5.0
        )
        
        if connection_result is not None:
            # Test the connection with a simple command
            battery = retry_command(
                lambda: drone.get_battery(),
                max_retries=2,
                timeout=2.0
            )
            
            if battery is not None:
                print(f"Connection successful! Battery: {battery}%")
                connection_successful = True
                break
            else:
                print("Could not get battery level. Connection may be unstable.")
    except Exception as e:
        print(f"Connection attempt {attempt+1} failed: {e}")
    
    # Wait before retry
    time.sleep(1)

if not connection_successful:
    print("WARNING: Could not establish reliable connection to the drone.")
    print("The program will continue, but some features may not work properly.")
    print("Make sure you are connected to the Tello's WiFi network.")
    print("You can try to reconnect by pressing 'C' during operation.")

# Turn on video stream with error handling
try:
    # Make sure any existing stream is off first
    try:
        drone.streamoff()
        time.sleep(0.5)  # Give the drone time to process
    except:
        pass  # Ignore errors if stream is already off
        
    print("Starting video stream...")
    drone.streamon()
    time.sleep(1)  # Wait for stream to initialize
except Exception as e:
    print(f"Error starting video stream: {e}")
    print("Trying to proceed anyway...")

# Face tracking function
def find_face(img):
    global face_tracking
    
    if not face_tracking:
        return [0, 0], 0, img  # Return default values when not tracking
    
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Detect faces
    faces = face_cascade.detectMultiScale(gray, 1.2, 8)
    
    face_list_center = []
    face_list_area = []
    
    # Process detected faces
    for (x, y, w, h) in faces:
        # Draw rectangle around face
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Calculate center point
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        
        # Mark center of face
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), cv2.FILLED)
        
        face_list_center.append([cx, cy])
        face_list_area.append(area)
    
    # Find the largest face if multiple are detected
    if len(face_list_area) != 0:
        i = face_list_area.index(max(face_list_area))
        return face_list_center[i], face_list_area[i], img
    else:
        return [0, 0], 0, img  # No face detected

# PID controller for tracking
def track_face(info, width, pid, p_error):
    global face_tracking
    
    if not face_tracking or info[0] == 0:
        return 0, p_error  # Don't track if disabled or no face
    
    # Calculate error from center
    error = info[0] - width // 2
    
    # PID calculation
    p = pid[0] * error
    i = pid[1] * (error + p_error)
    d = pid[2] * (error - p_error)
    
    # Calculate yaw velocity
    yaw_velocity = int(p + i + d)
    
    # Limit yaw velocity
    if yaw_velocity > S:
        yaw_velocity = S
    elif yaw_velocity < -S:
        yaw_velocity = -S
    
    return yaw_velocity, p_error  # Fixed: This line had a syntax error

# Aruco marker detection function
def find_aruco(img):
    global aruco_tracking
    
    if not aruco_tracking:
        return [0, 0], 0, img  # Return default values when not tracking
    
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Detect Aruco markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    # Process detected markers
    if ids is not None and len(ids) > 0:
        # Draw detected markers
        aruco.drawDetectedMarkers(img, corners, ids)
        
        # Get the center and area of the first detected marker
        c = corners[0][0]
        cx = int(np.mean([c[0][0], c[1][0], c[2][0], c[3][0]]))
        cy = int(np.mean([c[0][1], c[1][1], c[2][1], c[3][1]]))
        
        # Calculate area by finding the area of the quadrilateral
        area = cv2.contourArea(c.astype(np.float32))
        
        # Mark center of marker
        cv2.circle(img, (cx, cy), 5, (0, 255, 255), cv2.FILLED)
        
        return [cx, cy], area, img
    else:
        return [0, 0], 0, img  # No marker detected

# PID controller for tracking Aruco
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
    if yaw_velocity > S:
        yaw_velocity = S
    elif yaw_velocity < -S:
        yaw_velocity = -S
    
    return yaw_velocity, error

# Create a thread for video streaming
def video_thread():
    global pError, face_tracking, aruco_tracking, body_tracking
    
    # Variables to track landing sequence
    landing_initiated = False
    landing_start_time = 0
    
    while True:
        # Get frame from drone with error handling
        try:
            frame_read = drone.get_frame_read()
            if frame_read is None:
                # If frame_read is None, create a blank frame
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "NO VIDEO FEED", (190, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                frame = frame_read.frame
                if frame is None:
                    # If frame is None, create a blank frame
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(frame, "NO VIDEO FRAME", (190, 240), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        except Exception as e:
            # Create a blank frame with error message
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            error_msg = str(e)
            if len(error_msg) > 60:  # Truncate long error messages
                error_msg = error_msg[:60] + "..."
            cv2.putText(frame, f"VIDEO ERROR: {error_msg}", (10, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Process frame (you can add more processing here)
        frame = cv2.resize(frame, (640, 480))
        
        # Face detection and tracking (if enabled)
        info_face, area_face, img = [0, 0], 0, frame
        yaw_velocity_face = 0
        
        if face_tracking:
            info_face, area_face, img = find_face(frame)
            yaw_velocity_face, pError = track_face(info_face, 640, PID, pError)
        
        # Body detection and tracking (if enabled)
        info_body, area_body, img_body = [0, 0], 0, frame
        yaw_velocity_body = 0
        
        if body_tracking:
            # Update the shared frame for the body detection thread
            with body_lock:
                body_frame = frame.copy()
                
                # Get the latest results from the body detection thread
                info_body, area_body, img_body_result = body_result
                
                # Only update the display image if we have valid results
                if img_body_result is not None:
                    # Copy just the detection rectangles to our main frame
                    # This is more efficient than replacing the whole image
                    img_body = frame.copy()
                    mask = np.any(img_body_result != frame, axis=2)
                    img_body[mask] = img_body_result[mask]
            
            yaw_velocity_body, pError = track_body(info_body, 640, PID, pError)
            
            if not face_tracking:  # Only update img if face tracking hasn't already
                img = img_body
            # If both face and body tracking are on, the visualizations will be combined
        
        # Aruco marker detection and tracking (if enabled)
        info_aruco, area_aruco, img_aruco = [0, 0], 0, frame
        yaw_velocity_aruco = 0
        
        if aruco_tracking:
            # Use the image from previous detections
            if face_tracking or body_tracking:
                info_aruco, area_aruco, img_aruco = find_aruco(img)
            else:
                info_aruco, area_aruco, img_aruco = find_aruco(frame)
                
            yaw_velocity_aruco, pError = track_aruco(info_aruco, 640, PID, pError)
            img = img_aruco  # Update the image with Aruco markers
        
        # Determine which tracking to apply (priority: Aruco > Body > Face)
        if aruco_tracking and area_aruco > 0:
            # Display tracking status
            cv2.putText(img, f"Tracking Aruco: ({info_aruco[0]}, {info_aruco[1]})", (10, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(img, f"Area: {area_aruco:.1f}", (10, 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Calculate forward/backward based on area
            fb = 0
            up_down = 0
            
            # Larger thresholds for Aruco as they can be detected from further away
            if area_aruco > 10000:  # Close enough to land
                # If we're close enough to land, start landing sequence
                if not landing_initiated:
                    landing_initiated = True
                    landing_start_time = time.time()
                    cv2.putText(img, "LANDING SEQUENCE INITIATED", (150, 240), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Check if we've been in landing sequence for at least 1 second to avoid false triggers
                if landing_initiated and (time.time() - landing_start_time) > 1.0:
                    cv2.putText(img, "LANDING ON MARKER", (180, 280), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    # Stop tracking and land with error handling
                    try:
                        drone.send_rc_control(0, 0, 0, 0)  # Stop all movement
                        time.sleep(0.5)
                        drone.land()
                        aruco_tracking = False  # Disable tracking after landing
                        landing_initiated = False
                    except Exception as e:
                        print(f"Error during landing sequence: {e}")
                        cv2.putText(img, "LANDING ERROR", (180, 320), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                else:
                    # Hover in place for a moment
                    fb = 0
                    up_down = 0
            elif area_aruco > 4000:  # Too close, but not landing yet
                fb = -10  # Move backward slowly
                # Try to center vertically
                if info_aruco[1] < 240 - 50:  # Marker is too high
                    up_down = -10  # Move down
                elif info_aruco[1] > 240 + 50:  # Marker is too low
                    up_down = 10  # Move up
            elif area_aruco < 2000 and area_aruco > 0:  # Too far
                fb = 15  # Move forward
                landing_initiated = False  # Reset landing sequence if we move away
            
            # In tracking mode, update all controls with error handling
            if info_aruco[0] != 0:
                try:
                    drone.send_rc_control(0, fb, up_down, yaw_velocity_aruco)
                except Exception as e:
                    print(f"Error sending tracking commands: {e}")
                    cv2.putText(img, "COMMAND ERROR", (280, 320), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        elif body_tracking and area_body > 0:
            # Display tracking status
            cv2.putText(img, f"Tracking Body: ({info_body[0]}, {info_body[1]})", (10, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.putText(img, f"Area: {area_body}", (10, 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Calculate forward/backward based on area
            fb = 0
            up_down = 0
            
            # Body tracking thresholds - bodies are larger than faces
            if area_body > 30000:  # Too close
                fb = -20
            elif area_body < 15000 and area_body > 0:  # Too far
                fb = 20
                
            # Vertical centering
            if info_body[1] < 240 - 60:  # Body is too high in frame
                up_down = -10  # Move down
            elif info_body[1] > 240 + 60:  # Body is too low in frame
                up_down = 10  # Move up
            
            # In tracking mode, send commands with error handling
            if info_body[0] != 0:
                try:
                    drone.send_rc_control(0, fb, up_down, yaw_velocity_body)
                except Exception as e:
                    print(f"Error sending body tracking commands: {e}")
                    cv2.putText(img, "COMMAND ERROR", (280, 320), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        elif face_tracking and area_face > 0:
            # Display tracking status
            cv2.putText(img, f"Tracking Face: ({info_face[0]}, {info_face[1]})", (10, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(img, f"Area: {area_face}", (10, 100), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Calculate forward/backward based on area
            fb = 0
            if area_face > 8000:  # Too close
                fb = -20
            elif area_face < 3000 and area_face > 0:  # Too far
                fb = 20
            
            # In tracking mode, only update yaw and forward/backward with error handling
            if info_face[0] != 0:
                try:
                    drone.send_rc_control(0, fb, 0, yaw_velocity_face)
                except Exception as e:
                    print(f"Error sending face tracking commands: {e}")
                    cv2.putText(img, "COMMAND ERROR", (280, 320), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Add battery and other info to the frame
        try:
            battery = safe_drone_command(drone.get_battery, 0)
            height = safe_drone_command(drone.get_height, 0)
            temp = safe_drone_command(drone.get_temperature, 0)
            
            cv2.putText(img, f"Battery: {battery}%", (10, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(img, f"Height: {height}cm", (10, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(img, f"Temp: {temp}C", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add connection status indicator
            if battery > 0:
                connection_status = "CONNECTED"
                status_color = (0, 255, 0)  # Green
            else:
                connection_status = "CONNECTION ISSUE"
                status_color = (0, 0, 255)  # Red
                
            cv2.putText(img, connection_status, (480, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
        except Exception as e:
            # If there's any error getting drone stats, display error
            cv2.putText(img, "ERROR: Cannot read drone stats", (10, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(img, "CONNECTION ISSUE", (480, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Display tracking modes
        if face_tracking:
            cv2.putText(img, "Face Tracking: ENABLED", (10, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        elif body_tracking:
            cv2.putText(img, "Body Tracking: ENABLED", (10, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        elif aruco_tracking:
            cv2.putText(img, "Aruco Tracking: ENABLED", (10, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        else:
            cv2.putText(img, "Tracking: DISABLED", (10, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Display help text for controls
        cv2.putText(img, "Controls:", (10, 400), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(img, "T: Takeoff | L: Land | Esc: Emergency Stop", (10, 420), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, "Arrow Keys: Forward/Back/Left/Right", (10, 440), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, "W/S: Up/Down | A/D: Rotate Left/Right", (10, 460), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, "F: Face Tracking | B: Body Tracking | G: Aruco Landing", (10, 480), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Display the frame
        cv2.imshow("Tello Drone Feed", img)
        
        # Break loop with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Create video thread but don't start it yet
# We'll start it in the main execution block with retry mechanism
video_thread = threading.Thread(target=video_thread)
video_thread.daemon = True

# Create and start the body detection thread
body_detection_thread = threading.Thread(target=body_detection_thread)
body_detection_thread.daemon = True
body_detection_thread.start()

# Main control loop
def control_loop():
    # Initialize movement variables
    left_right, forward_backward, up_down, yaw = 0, 0, 0, 0
    global face_tracking, aruco_tracking
    
    # Variables to handle key press toggle
    f_key_pressed = False
    g_key_pressed = False
    
    while True:
        # Only process manual controls if no tracking is enabled
        if not face_tracking and not aruco_tracking:
            # Reset movement values
            left_right, forward_backward, up_down, yaw = 0, 0, 0, 0
            
            # Forward/Backward movement
            if keyboard.is_pressed('up'):
                forward_backward = S
            elif keyboard.is_pressed('down'):
                forward_backward = -S
            
            # Left/Right movement
            if keyboard.is_pressed('left'):
                left_right = -S
            elif keyboard.is_pressed('right'):
                left_right = S
            
            # Up/Down movement
            if keyboard.is_pressed('w'):
                up_down = S
            elif keyboard.is_pressed('s'):
                up_down = -S
            
            # Yaw (rotation) movement
            if keyboard.is_pressed('a'):
                yaw = -S
            elif keyboard.is_pressed('d'):
                yaw = S
            
            # Send RC control command to the drone (only when not tracking)
            drone.send_rc_control(left_right, forward_backward, up_down, yaw)
        
        # These commands work regardless of tracking status
        
        # Takeoff
        if keyboard.is_pressed('t'):
            # Takeoff with non-blocking function
            if not safe_takeoff():
                # If takeoff failed, show message on screen
                print("Consider reconnecting drone with 'C' key")
            time.sleep(0.5)  # Prevent multiple commands
        
        # Land
        if keyboard.is_pressed('l'):
            # Land with error handling
            try:
                print("Landing!")
                drone.land()
                print("Landing successful!")
            except Exception as e:
                print(f"Landing error: {e}")
                # Try emergency stop if normal landing fails
                try:
                    print("Attempting emergency stop...")
                    drone.emergency()
                except:
                    print("Emergency stop also failed.")
            time.sleep(0.5)
        
        # Emergency stop
        if keyboard.is_pressed('esc'):
            # Emergency stop with error handling
            try:
                print("Emergency Stop!")
                drone.emergency()
                print("Emergency stop executed!")
            except Exception as e:
                print(f"Emergency stop error: {e}")
                print("Trying alternate emergency measures...")
                # Try to force the drone to land any way possible
                try:
                    drone.land()
                except:
                    pass
            break
        
        # Toggle face tracking with F key
        if keyboard.is_pressed('f') and not f_key_pressed:
            # Disable Aruco tracking if enabling face tracking
            if aruco_tracking:
                aruco_tracking = False
            
            face_tracking = not face_tracking
            f_key_pressed = True
            print(f"Face tracking: {'ON' if face_tracking else 'OFF'}")
            
            # If disabling face tracking, stop the drone movement
            if not face_tracking:
                drone.send_rc_control(0, 0, 0, 0)
            
            time.sleep(0.3)  # Debounce delay
        elif not keyboard.is_pressed('f'):
            f_key_pressed = False
            
        # Toggle Aruco marker tracking with G key
        if keyboard.is_pressed('g') and not g_key_pressed:
            # Disable face tracking if enabling Aruco tracking
            if face_tracking:
                face_tracking = False
            
            aruco_tracking = not aruco_tracking
            g_key_pressed = True
            print(f"Aruco tracking and landing: {'ON' if aruco_tracking else 'OFF'}")
            
            # If disabling Aruco tracking, stop the drone movement
            if not aruco_tracking:
                drone.send_rc_control(0, 0, 0, 0)
            
            time.sleep(0.3)  # Debounce delay
        elif not keyboard.is_pressed('g'):
            g_key_pressed = False
            
        # Try to reconnect drone with C key
        c_key_pressed = False
        if keyboard.is_pressed('c') and not c_key_pressed:
            c_key_pressed = True
            print("Attempting to reconnect drone...")
            reconnect_drone()
            time.sleep(0.5)  # Debounce delay
        elif not keyboard.is_pressed('c'):
            c_key_pressed = False
        
        # Short sleep to prevent excessive commands
        time.sleep(0.05)

# Add a function to reconnect video if needed
def reconnect_video():
    print("Attempting to reconnect video stream...")
    try:
        drone.streamoff()
        time.sleep(1)
        drone.streamon()
        time.sleep(2)
        frame_read = drone.get_frame_read()
        if frame_read and frame_read.frame is not None:
            print("Video stream reconnected successfully!")
            return True
        else:
            print("Failed to reconnect video stream.")
            return False
    except Exception as e:
        print(f"Error reconnecting video: {e}")
        return False

# Main execution
try:
    print("Tello Keyboard Control Started!")
    print("Controls:")
    print("T: Takeoff")
    print("L: Land")
    print("ESC: Emergency Stop")
    print("Arrow Keys: Forward/Back/Left/Right")
    print("W/S: Up/Down")
    print("A/D: Rotate Left/Right")
    print("F: Toggle Face Tracking")
    print("B: Toggle Body Tracking")
    print("G: Toggle Aruco Marker Tracking & Landing")
    print("R: Reconnect Video Stream")
    print("C: Reconnect Drone")
    print("Q: Quit")
    
    # Start video thread with retry mechanism
    retry_count = 0
    while retry_count < 3:
        try:
            video_thread.start()
            break  # If successful, exit the retry loop
        except Exception as e:
            print(f"Error starting video thread: {e}")
            retry_count += 1
            if retry_count < 3:
                print(f"Retry attempt {retry_count}...")
                time.sleep(2)
                # Try to fix the connection
                reconnect_video()
            else:
                print("Failed to start video thread after multiple attempts. Proceeding with control loop only.")
    
    # Add special handling for the 'r' key to reconnect video and 'c' key to reconnect drone
    def check_reconnect():
        while True:
            if keyboard.is_pressed('r'):
                print("Reconnect video requested")
                reconnect_video()
                time.sleep(0.5)  # Debounce
            
            if keyboard.is_pressed('c'):
                print("Reconnect drone requested")
                reconnect_drone()
                time.sleep(0.5)  # Debounce
                
            time.sleep(0.1)
    
    # Start reconnect checker thread
    reconnect_thread = threading.Thread(target=check_reconnect)
    reconnect_thread.daemon = True
    reconnect_thread.start()
    
    # Start control loop
    control_loop()
    
except KeyboardInterrupt:
    print("Program stopped by user")
    
finally:
    # Clean up with extra error handling
    print("Shutting down...")
    try:
        drone.streamoff()
    except:
        pass
    try:
        drone.end()
    except:
        pass
    try:
        cv2.destroyAllWindows()
    except:
        pass
    
    # Force close UDP connections
    import socket
    for port in [8889, 8890, 11111]:  # Tello communication ports
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('0.0.0.0', port))
            sock.close()
            print(f"Released port {port}")
        except:
            pass
    
    print("Cleanup complete.")