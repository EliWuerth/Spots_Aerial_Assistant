import cv2
import numpy as np
from djitellopy import Tello
import cv2.aruco as aruco
import time

class TelloMarkerController:
    def __init__(self,tello):
        # Initialize the Tello drone
        self.tello = tello
        
        # Load the ArUco dictionary and create the detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_id_to_detect = 0  # Change this to the ID of the marker you want to detect

    def detect_marker(self, frame):
        if frame is None or len(frame.shape) < 3:
            print("Invalid frame received.")
            return None, None
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        return corners, ids

    def move_to_marker(self, frame):
        corners, ids = self.detect_marker(frame)

        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                # Draw the detected markers
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Calculate the center of the marker
                c = corners[i][0]
                center_x = int((c[0][0] + c[2][0]) / 2)
                center_y = int((c[0][1] + c[2][1]) / 2)
                print(f"Center: ({center_x}, {center_y})")

                # Move the drone towards the marker
                if ids[i][0] == self.marker_id_to_detect:  # Assuming the origin marker has the specified ID
                    # Calculate the distance to the center of the marker
                    dx = center_x - 320  # Center of the frame (640x480)
                    dy = center_y - 240

                    # Clamp dx and dy to a maximum movement range
                    max_move = 50  # Maximum movement in cm
                    dx = int(max(max_move, min(max_move, dx // 10))) # Scale and clamp dx
                    dy = int(max(max_move, min(max_move, dy // 10)))  # Scale and clamp dy
                    dx=abs(dx)
                    dy=abs(dy)

                    # Calculate speed based on distance
                    distance = np.sqrt(dx**2 + dy**2)
                    speed = min(100, max(20, int(distance / 10)))  # Speed between 20 and 100 cm/s

                    # Log the values for debugging
                    print(f"Marker ID: {ids[i][0]}, Center: ({center_x}, {center_y}), dx: {dx}, dy: {dy}, speed: {speed}")

                    # Move the drone
                    self.tello.go_xyz_speed(dx, dy, 0, 10)  # Send the command

        return frame  # Return the frame for display
    
    def land_on_marker(self):
        # Start the drone
        self.tello.takeoff()
        time.sleep(2)  # Wait for the drone to take off

        while True:
            frame = self.tello.get_frame_read().frame
            updated_frame = self.move_to_marker(frame)

            # Display the frame
            cv2.imshow("Tello Video Stream", updated_frame)

            # Check if the drone is close enough to the marker to land
            if self.is_close_to_marker(updated_frame):
                print("Landing on marker...")
                self.tello.land()
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    def is_close_to_marker(self, frame):
        # Implement logic to determine if the drone is close enough to the marker
        # For example, you can check the size of the detected marker or the distance
        corners, ids = self.detect_marker(frame)
        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                if ids[i][0] == self.marker_id_to_detect:
                    # Check the size of the marker or its position
                    return True  # Return True if close enough
        return False  # Return False if not close enough

    def cleanup(self):
        self.tello.set_video_direction(self.tello.CAMERA_FORWARD)
        self.tello.end()
        cv2.destroyAllWindows()
        exit()

if __name__ == "__main__":
    controller = TelloMarkerController()
    try:
        controller.run()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()