import cv2
import numpy as np
from djitellopy import tello

class TelloApp:
    def __init__(self):
        # Initialize the Tello drone
        self.tello = tello.Tello()
        self.tello.connect()
        print("Tello connected!")

        # Start video stream
        self.tello.streamon()
        print("Video stream started.")
        self.camera_down = True
        self.frame_read = self.tello.get_frame_read()
        print("Battery percentage:", self.tello.get_battery())

        # Load the ArUco dictionary and create the detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_id_to_detect = 0  # Change this to the ID of the marker you want to detect

    def detect_marker(self, frame):
        if frame is None or len(frame.shape) < 3:
            print("Invalid frame received.")
            return False, None
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
        
        if ids is not None and self.marker_id_to_detect in ids:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            return True, corners
        return False, None
    
    def move_to_marker(self, corners):
        # Calculate the center of the detected marker
        center_x = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
        center_y = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)

        # Get the frame dimensions
        frame_width = 320  # Adjust based on your resizing
        frame_height = 240  # Adjust based on your resizing

        # Calculate the offset from the center of the frame
        offset_x = center_x - (frame_width / 2)
        offset_y = center_y - (frame_height / 2)

        # Define movement parameters
        move_threshold = 20  # Threshold for movement
        move_distance = 20  # Base distance to move
        speed_factor = 0.5  # Speed adjustment factor

        # Move the drone based on the offset
        if abs(offset_x) > move_threshold:  # Threshold to avoid small movements (Horizontal)
            move_x = int(move_distance * (abs(offset_x) / (frame_width / 2)) * speed_factor)
            if offset_x > 0:
                self.tello.move_right(move_x)  # Move right
            else:
                self.tello.move_left(move_x)  # Move left

        if abs(offset_y) > move_threshold:  # Threshold to avoid small movements (Vertical)
            move_y = int(move_distance * (abs(offset_y) / (frame_height / 2)) * speed_factor)
            if offset_y > 0:
                self.tello.move_down(move_y)  # Move down
            else:
                self.tello.move_up(move_y)  # Move up

        # Move forward if the marker is too far away
        if abs(offset_x) < move_threshold and abs(offset_y) < move_threshold:
            self.tello.move_forward(20)  # Move forward if close to the center

    def run(self):
        print("Taking off...")
        self.tello.takeoff()
        self.tello.set_video_direction(self.tello.CAMERA_DOWNWARD)
        # self.tello.set_video_resolution(self.tello.RESOLUTION_720P)
        
        try:
            while True:
                img = self.frame_read.frame

                if self.camera_down:
                    img = cv2.resize(img, [320, 240])
                    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

                marker_detected, corners = self.detect_marker(img)
                
                # Display the frame
                cv2.imshow("Tello Video Feed", img)

                marker_detected, corners = self.detect_marker(img)

                if (marker_detected):
                    self.move_to_marker(corners)
                    print("Moving to marker center...")

                    # Check if the drone is close enough to the marker
                    if abs(corners[0][0][0][0] - corners[0][0][2][0]) < 50 and abs(corners[0][0][0][1] - corners[0][0][2][1]) < 50:
                        print("Marker center reached! Landing...")
                        self.tello.land()
                        break

                # Exit on 'q' key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Landing due to user command.")
                    self.tello.land()
                    break

        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.tello.end()
            
            cv2.destroyAllWindows()
            print("Program ended.")

if __name__ == "__main__":
    app = TelloApp()
    app.run()
