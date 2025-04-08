import cv2
import numpy as np
from djitellopy import Tello
import cv2.aruco as aruco
import threading
import time

S = 60  # Default speed
aruco_tracking = False 

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
    return None  # All attempts failed

class CenterMarkerController:
    def __init__(self, tello):
        self.tello = tello
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_tracking = False  # Initialize tracking state

    def find_aruco(self, img):
        global aruco_tracking
        
        if not aruco_tracking:
            return [0, 0], 0, img  # Return default values when not tracking
        
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Check if the gray image is valid
        if gray is None or gray.size == 0:
            print("Error: The gray image is empty.")
            return [0, 0], 0, img  # Return default values
        
        # Detect Aruco markers
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, self.aruco_params)
        
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
    
    def start_tracking(self):
        self.aruco_tracking = True  # Enable tracking

    def stop_tracking(self):
        self.aruco_tracking = False  # Disable tracking

    # PID controller for tracking Aruco
    def track_aruco(self, info, width, pid, p_error):
        if not self.aruco_tracking or info[0] == 0:
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

    def safe_land(self):
        """Safely land the drone with retry logic."""
        print("Landing! (with timeout protection)")
        land_success = retry_command(lambda: self.tello.land())
        if land_success:
            print("Landing successful!")
        else:
            print("Landing failed.")

# Example usage
if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print("Battery percentage:", tello.get_battery())
    
    controller = CenterMarkerController(tello)
    controller.start_tracking()
    
    # Simulate a video feed (replace with actual video feed in practice)
    while True:
        # Here you would get the frame from the drone
        # For example: img = tello.get_frame_read().frame
        img = np.zeros((480, 640, 3), dtype=np.uint8)  # Placeholder for an actual image
        
        # Find ArUco markers
        info, area, img_with_markers = controller.find_aruco(img)
        
        # Display the image with markers (for testing)
        cv2.imshow("Aruco Marker", img_with_markers)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    controller.safe_land()
    tello.end()
    cv2.destroyAllWindows()