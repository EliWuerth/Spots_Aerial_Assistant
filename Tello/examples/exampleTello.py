import cv2
import time
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
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

def land():
    drone.land()

def move_forward():
    drone.move_forward(30)  # Move forward 30 cm

def move_up():
    drone.move_up(30)  # Move up 30 cm

def move_down():
    drone.move_down(30)  # Move down 30 cm

def move_backward():
    drone.move_back(30)  # Move backward 30 cm

def move_left():
    drone.move_left(30)  # Move left 30 cm

def move_right():
    drone.move_right(30)  # Move right 30 cm

def rotate_clockwise():
    drone.rotate_clockwise(45)  # Rotate clockwise 45 degrees

def rotate_counter_clockwise():
    drone.rotate_counter_clockwise(45)  # Rotate counter-clockwise 45 degrees

def emergency_stop():
    drone.emergency()  # Use emergency stop

def send_tello_command(command, retries=3, timeout=5):
    for i in range(retries):
        try:
            print(f"Sending command: {command} (Attempt {i+1})")
            response = drone.send_command_with_return(command, timeout=timeout)
            if response:
                print(f"Command successful: {response}")
                return response
        except Exception as e:
            print(f"Error sending command {command}: {e}")

    print(f"Command {command} failed after {retries} attempts.")
    return None

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
                #############################
                #      Problem Area         #
                #############################
                # front camera is to close and can't see the marker so  it  switches to the bottom camera but its not over top the marker so the bottom camera can't see the marker
                # Not as big of a problem anymore, but still is troubling 
                
                # Lost marker handling
                if time.time() - last_seen_time > 2 and not marker_lost:  # 2 seconds without marker
                    marker_lost = True  # Prevent re-execution
                    print("Marker lost - hovering")
                    
                    # Hover in place
                    drone.send_rc_control(0, 0, 0, 0)
                    time.sleep(1)

                    # Hover in place and try a slight descent before switching
                    drone.send_rc_control(0, 0, 20, 0)  # Gradually ascend
                    time.sleep(1)

                    drone.send_rc_control(0, 10, 0, 0)  # Gradually descend
                    time.sleep(1)

                    # Re-check with front camera before switching
                    img = drone.get_frame_read().frame
                    aruco_info, area, processed_frame = find_aruco(img)
                    
                    # Marker still not found? Switch cameras.
                    if aruco_info[0] == 0:
                        # drone.set_video_direction(drone.CAMERA_DOWNWARD)
                        processed_frame = bottom_camera_and_land(img)

            # Display the processed frame
            
            # if camera_down != False:
            #     processed_frame = cv2.rotate(processed_frame, cv2.ROTATE_90_CLOCKWISE)
            # cv2.imshow("Drone Feed", processed_frame)
            # cv2.waitKey(1)

            # Check for GUI events
            # Ensure root exists before updating
            if root.winfo_exists():
                root.update_idletasks()
                root.update()
            else:
                print("tkinter GUI closed. Stopping GUI updates.")
                break  # Stop loop if the window is destroyed

        except Exception as e:
            print(f"Error updating GUI line 202: {e}")
            drone.land()
            break  # Break out of the loop to prevent infinite errors

    pError = 0  # Reset PID error
    aruco_tracking = False
    cv2.destroyAllWindows()

def search_for_aruco_bottom_camera():
    global aruco_tracking

    # Switch to the bottom camera
    print("Switching to bottom camera...")
    drone.set_video_direction(drone.CAMERA_DOWNWARD)

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
                drone.send_rc_control(0, 0, 0, 0)  # Stop the drone
                drone.land()  # Command the drone to land
                cv2.destroyAllWindows()  # Close any OpenCV windows
                
                return aruco_info, area

            # If no marker is found, rotate and move in a search pattern
            print("Marker not found, searching...")
            drone.send_rc_control(0, 0, 0, 0)  # Stop any movement

            rotation_successful = False
            while not rotation_successful:
                drone.rotate_clockwise(int(search_step))  # Rotate the drone
                time.sleep(1)  # Allow time for the rotation to complete
                if check_imu():  # Check if the IMU is stable
                    rotation_successful = True
                else:
                    print("IMU instability detected, retrying rotation...")

            print("Moving forward...")
            drone.send_rc_control(0, search_distance, 50, 0)  # Move forward
            time.sleep(1)  # Wait for the movement to complete
        except Exception as e:
            print(f"Error during search: {e}")  # Handle any exceptions that occur during the search
        
        attempt += 1  # Increment the attempt counter

    print("Max search attempts reached. Returning to hover mode.")
    drone.send_rc_control(0, 0, 0, 0)  # Stop movement and hover
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
    
    while not landed:  # Loop until the drone has landed
        try:
            # Get the frame from the drone's bottom camera
            frame = drone.get_frame_read().frame
            if frame is None or frame.size == 0:
                print("Warning: Empty frame received from drone. Skipping...")
                continue  # Skip to the next iteration if the frame is empty

            # Resize the frame for processing
            frame = cv2.resize(frame, (640, 480))
            aruco_info, area, processed_frame = find_aruco(frame)  # Find ArUco markers in the frame

            # Marker tracking logic
            if aruco_info[0] != 0:  # If the marker is found
                print("Marker found, landing...")
                drone.land()
                # cx, cy = aruco_info  # Get the coordinates of the marker
                # error_x = cx - frame.shape[1] // 2  # Calculate horizontal error
                # error_y = cy - frame.shape[0] // 2  # Calculate vertical error

                # # PID calculations for yaw control
                # yaw_velocity, pError = track_aruco(aruco_info, frame.shape[1], PID, pError)
                
                # # Forward control (approach marker)
                # forward_speed = max(30, int(SPEED * (1 - min(area / (LANDING_THRESHOLD * 1.5), 1))))  # Calculate forward speed based on area of the marker
                
                # # Move towards the marker if the error is significant
                # if abs(error_x) > DISTANCE_THRESHOLD:
                #     drone.send_rc_control(0, forward_speed // 2, 0, yaw_velocity // 2)  # Control drone to move towards the marker
                #     time.sleep(1)  # Wait for the movement to take effect

                # # If close enough to the marker, initiate landing
                # if area > LANDING_THRESHOLD:
                #     print("Marker detected and close enough. Landing!")
                #     drone.land()  # Command the drone to land
                #     landed = True  # Set landed flag to True
                #     break  # Exit the loop when landing is complete
            elif aruco_info[0] == 0:
                # If the marker is lost, start searching for it
                print("Marker not found, starting search...")
                aruco_info, area = search_for_aruco_bottom_camera()  # Call the search function to find the marker

                # print("Found marker after search!")
                # drone.send_rc_control(0, 0, 0, 0)  # Stop the drone
                # time.sleep(1)  # Wait for a moment
                # drone.land()  # Command the drone to land
                # landed = True  # Set landed flag to True
                # cv2.destroyAllWindows()  # Close any OpenCV windows
                # break  # Exit the loop

            print("show vid")  # Placeholder for showing video feed

        except Exception as e:
            print(f"Error updating GUI line 369: {e}")  # Handle any exceptions that occur during the process
            drone.land()  # Command the drone to land in case of an error
            break  # Break out of the loop to prevent infinite errors

    pError = 0  # Reset PID error for future use
    aruco_tracking = False  # Disable ArUco tracking after landing
    
# tkinter GUI
def create_gui():
    global battery_label, temperature_label, video_label
    root = tk.Tk()
    root.title("Tello Drone Controller")
    
    # Load background image
    background_image = Image.open("Images/Gold-Brayer2.png")  # Replace with your image file
    background_image = background_image.resize((800, 800), Image.ANTIALIAS)  # Resize to fit the window
    background_photo = ImageTk.PhotoImage(background_image)

    # Create a canvas to hold the background image
    canvas = tk.Canvas(root, width=800, height=800, highlightcolor='gold')
    canvas.pack(fill="both", expand=True)
    canvas.create_image(0, 0, image=background_photo, anchor="nw")

    # Create a frame for the controls
    control_frame = tk.Frame(root, bd=1)
    control_frame.place(relx=0.5, rely=0.85, anchor="center")

    # Video stream placeholder
    video_frame = tk.Frame(root, width=640, height=480, highlightbackground='yellow')
    video_frame.place(relx=0.5, rely=0.45, anchor="center")
    video_label = tk.Label(video_frame)
    video_label.pack(expand=True, fill=tk.BOTH)

    button_options = {'padx': 8, 'pady': 8, 'width': 10}

    # Add buttons for vertical movement
    up_button = tk.Button(control_frame, text="Up", command=move_up, **button_options)
    up_button.grid(row=0, column=1)

    down_button = tk.Button(control_frame, text="Down", command=move_down, **button_options)
    down_button.grid(row=2, column=1)

    # Add buttons for horizontal movement
    left_button = tk.Button(control_frame, text="Left", command=move_left, **button_options)
    left_button.grid(row=1, column=4)

    right_button = tk.Button(control_frame, text="Right", command=move_right, **button_options)
    right_button.grid(row=1, column=6)

    # Add buttons for forward and backward movement
    forward_button = tk.Button(control_frame, text="Forward", command=move_forward, **button_options)
    forward_button.grid(row=0, column=5)

    backward_button = tk.Button(control_frame, text="Backward", command=move_backward, **button_options)
    backward_button.grid(row=2, column=5)

    # Add new buttons for rotation
    counter_clockwise_button = tk.Button(control_frame, text="Spin Left", command=rotate_counter_clockwise, **button_options)
    counter_clockwise_button.grid(row=1, column=0)

    clockwise_button = tk.Button(control_frame, text="Spin Right", command=rotate_clockwise, **button_options)
    clockwise_button.grid(row=1, column=2)

    # Add buttons for takeoff, landing, and "Go To" button controls
    takeoff_button = tk.Button(control_frame, text="Take Off", highlightbackground='green3', command=safe_takeoff, **button_options)
    takeoff_button.grid(row=0, column=3)

    land_button = tk.Button(control_frame, text="Land", highlightbackground='yellow', command=land, **button_options)
    land_button.grid(row=1, column=3)

    emergency_stop_button = tk.Button(control_frame, text="Emergency Stop", highlightbackground='red3', command=emergency_stop, **button_options)
    emergency_stop_button.grid(row=3, column=3)


    go_to_button = tk.Button(control_frame, text="Go to ArUco and Land", highlightbackground='lightblue', command=go_to_aruco_and_land, **button_options)
    go_to_button.grid(row=2, column=3)

    # Battery status label
    battery_label = tk.Label(root, text="Battery: 100%", bg="white")
    battery_label.place(relx=0.5, rely=0.05, anchor="center")  # Center the battery label at the top

    temperature_label = tk.Label(root, text="Temperature: 25°C")
    temperature_label.place(relx=0.9, rely=0.05, anchor="center")

    # Camera switch button
    camera_button = tk.Button(root, text="Switch Camera", highlightbackground="gold", command=set_camera_direction, **button_options)
    camera_button.place(relx=0.5, rely=0.1, anchor="center")  # Center the camera button below the battery label


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

# def process_video_gui():
#     try:
#         # Get the frame from the drone's camera
#         frame = drone.get_frame_read().frame
#         frame = cv2.resize(frame, (640, 480))

#         #Process ArUco detection
#         aruco_info, area, processed_frame = find_aruco(frame)

#         # Convert to RGB and then ImageTk format
#         processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
#         img = Image.fromarray(processed_frame)
#         imgtk = ImageTk.PhotoImage(image=img)

#         #Update the video_label with the new frame 
#         video_label.imgtk = imgtk   #Prevent garbage collection
#         video_label.config(image=imgtk)

#         #Schedule the next frame update
#         root.after(33, process_video_gui) #Roughly 30 fps

#     except Exception as e:
#         print(f"Video Processing Error: {e}")


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

        if root.winfo_exists():
            root.update_idletasks()
            root.update()
        else:
            print("tkinter GUI closed. Stopping GUI updates.")
            break  # Stop loop if the window is destroyed
    
    pError = 0  # Reset PID error before starting tracking loop
    cv2.destroyAllWindows()

# === Only run the GUI when exampleTello.py is executed directly ===
if __name__ == "__main__":
    # Initialize drone
    drone.connect()
    drone.streamon()
    drone.set_video_direction(drone.CAMERA_FORWARD)

    # Create GUI
    root = create_gui()
    update_drone_status()

    # Start GUI-integrated video stream
    def process_video_gui():
        try:
            frame = drone.get_frame_read().frame
            if frame is None or frame.size == 0:
                print("Empty frame received.")
                root.after(33, process_video_gui)
                return

            frame = cv2.resize(frame, (640, 480))
            aruco_info, area, processed_frame = find_aruco(frame)

            # Rotate if bottom camera is active
            if camera_down:
                processed_frame = cv2.rotate(processed_frame, cv2.ROTATE_90_CLOCKWISE)

            # === FIX BLUE TINGE HERE ===
            processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
            b, g, r = cv2.split(processed_frame)
            r = cv2.addWeighted(r, 1.1, r, 0, 0)  # Boost red
            g = cv2.addWeighted(g, 1.05, g, 0, 0)  # Slightly boost green
            b = cv2.addWeighted(b, 0.9, b, 0, 0)   # Slightly reduce blue
            processed_frame = cv2.merge((r, g, b))

            # Convert to ImageTk format
            img = Image.fromarray(processed_frame)
            imgtk = ImageTk.PhotoImage(image=img)

            video_label.imgtk = imgtk
            video_label.config(image=imgtk)

        except Exception as e:
            print(f"Video Processing Error: {e}")

        root.after(33, process_video_gui)

    # Start the video loop and GUI
    process_video_gui()
    root.mainloop()
