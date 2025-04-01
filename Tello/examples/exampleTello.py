import cv2
import time
import numpy as np
import tkinter as tk
import cv2.aruco as aruco
from djitellopy import Tello

# Constants
SPEED = 60
PID = [0.3, 0.0005, 0.1]  # Reduced proportional gain
LANDING_THRESHOLD = 1500  # Increased to avoid premature landing
DISTANCE_THRESHOLD = 20  # Distance threshold for moving forward

# Global variables
drone = Tello()
face_tracking = False
body_tracking = False
aruco_tracking = False
camera_down = False
pError = 0
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
aruco_params = aruco.DetectorParameters()

# Takeoff function
def safe_takeoff():
    try:
        drone.takeoff()
        print("Takeoff successful!")
        return True
    except Exception as e:
        print(f"Takeoff error: {e}")
        return False

# Function to switch camera direction
def set_camera_direction():
    global camera_down
    if camera_down:
        camera_down = False
        drone.set_video_direction(drone.CAMERA_FORWARD)
    else:
        camera_down = True
        drone.set_video_direction(drone.CAMERA_DOWNWARD)

# Toggle tracking modes
def toggle_tracking(mode):
    global face_tracking, aruco_tracking
    if mode == 'aruco':
        aruco_tracking = not aruco_tracking
        print(f"Aruco tracking: {'ON' if aruco_tracking else 'OFF'}")

# ArUco detection function
def find_aruco(img):
    global aruco_tracking

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

#Function to approach and land on the ArUco marker
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
            frame = drone.get_frame_read().frame
            if frame is None:
                continue
                
            frame = cv2.resize(frame, (640, 480))
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
                forward_speed = max(30, int(SPEED * (1 - min(area / (LANDING_THRESHOLD * 1.5), 1))))
                
                # Vertical adjustment based on vertical position
                vertical_speed = int(PID[0] * error_y)
                
                # Only move forward if marker is roughly centered (error within threshold)
                if abs(error_x) > DISTANCE_THRESHOLD:
                    drone.send_rc_control(0, forward_speed, -vertical_speed, yaw_velocity)
                    time.sleep(1)

                print(f"Area: {area}, Forward: {forward_speed}, Vertical: {vertical_speed}, Yaw: {yaw_velocity}")
                
            else:
                # Lost marker handling
                if time.time() - last_seen_time > 2 and not marker_lost:  # 2 seconds without marker
                    marker_lost = True  # Prevent re-execution
                    print("Marker lost - hovering")
                    
                    # Hover in place
                    drone.send_rc_control(0, 0, 0, 0)
                    time.sleep(1)

                    # Set camera direction downward
                    drone.set_video_direction(drone.CAMERA_DOWNWARD)
                    time.sleep(1)  # Allow time for camera switch

                    # Re-check for the marker from top view
                    frame = drone.get_frame_read().frame
                    if frame is not None:
                        frame = cv2.resize(frame, (640, 480))
                        aruco_info, area, processed_frame = find_aruco(frame)

                        cx, cy = aruco_info
                        error_x = cx - frame.shape[1] // 2
                        error_y = cy - frame.shape[0] // 2

                        # Check if the marker is centered and within landing threshold
                        if area > LANDING_THRESHOLD and abs(aruco_info[0] - frame.shape[1] // 2) < 20 and abs(aruco_info[1] - frame.shape[0] // 2) < 20:
                            print("Marker detected directly below - landing")
                            drone.send_rc_control(0, 0, 0, 0)  # Stop motion
                            time.sleep(0.5)
                            drone.land()
                            aruco_tracking = False
                            return  # Exit function after landing
                        else:
                            print("Marker not directly below - hovering")
                            vertical_speed = int(PID[0] * error_y)

                            drone.send_rc_control(0, 0, 0, 0)  # Hover

            # Display the processed frame
            processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
            if camera_down != False:
                processed_frame = cv2.rotate(processed_frame, cv2.ROTATE_90_CLOCKWISE)
            cv2.imshow("Drone Feed", processed_frame)
            
            # Check for GUI events
            # Ensure root exists before updating
            if root.winfo_exists():
                root.update_idletasks()
                root.update()
            else:
                print("Tkinter GUI closed. Stopping GUI updates.")
                break  # Stop loop if the window is destroyed
        except Exception as e:
            print(f"Error updating GUI: {e}")
            drone.land()
            break  # Break out of the loop to prevent infinite errors

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            drone.streamoff()
            break

    pError = 0  # Reset PID error
    aruco_tracking = False
    cv2.destroyAllWindows()

# Tkinter GUI
def create_gui():
    global battery_label, temperature_label
    root = tk.Tk()
    root.title("Tello Drone Controller")
    
    tk.Button(root, text="Takeoff", command=safe_takeoff).pack()
    tk.Button(root, text="Land", command=drone.land).pack()
    tk.Button(root, text="Forward", command=lambda: drone.move_forward(30)).pack()
    tk.Button(root, text="Backward", command=lambda: drone.move_back(30)).pack()
    tk.Button(root, text="Left", command=lambda: drone.move_left(30)).pack()
    tk.Button(root, text="Right", command=lambda: drone.move_right(30)).pack()
    tk.Button(root, text="Up", command=lambda: drone.move_up(30)).pack()
    tk.Button(root, text="Down", command=lambda: drone.move_down(30)).pack()
    tk.Button(root, text="Rotate Left", command=lambda: drone.rotate_counter_clockwise(30)).pack()
    tk.Button(root, text="Rotate Right", command=lambda: drone.rotate_clockwise(30)).pack()
    tk.Button(root, text="Toggle Aruco Tracking", command=lambda: toggle_tracking('aruco')).pack()
    tk.Button(root, text="Go to ArUco and Land", command=go_to_aruco_and_land).pack()
    tk.Button(root, text="Switch Camera", highlightbackground="gold", command=lambda: set_camera_direction()).pack()
    
    # Labels to display battery status and temperature
    battery_label = tk.Label(root, text="Battery: 100%")
    battery_label.pack(pady=5)
    
    temperature_label = tk.Label(root, text="Temperature: 25°C")
    temperature_label.pack(pady=5)

    return root

# Function to update battery status and temperature
def update_drone_status():
    try:
        battery_label.config(text=f"Battery: {drone.get_battery()}%")  # Update battery label
        temperature_label.config(text=f"Temperature: {drone.get_temperature()}°C")  # Update temperature label
    except Exception as e:
        print(f"Error retrieving drone status: {e}")
    
    # Schedule the next update
    root.after(100, update_drone_status)  # Update every second

# Main loop for processing video stream
def process_video():
    global pError
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)
    while True:
        # Get the frame from the drone's camera
        frame = drone.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))  # Resize for processing

        # Find ArUco markers
        aruco_info, area, processed_frame = find_aruco(frame)

        # Display the processed frame
        processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
        if camera_down != False:
            processed_frame = cv2.resize(processed_frame, (640, 480))
            processed_frame = cv2.rotate(processed_frame, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow("Drone Feed", processed_frame)

        # Check for GUI events
        root.update_idletasks()
        root.update()

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    pError = 0  # ✅ Reset PID error before starting tracking loop
    cv2.destroyAllWindows()

# Initialize drone
drone.connect()
drone.streamon()
drone.set_video_direction(drone.CAMERA_FORWARD)

# Create GUI
root = create_gui()
update_drone_status()

# Start video processing
process_video()

# Close the GUI
root.destroy()
