import cv2
import numpy as np
from djitellopy import Tello
import tkinter as TK
from PIL import Image, ImageTk
from threading import Thread
import cv2.aruco as aruco
import time

# Constants
SPEED = 60
PID = [0.3, 0.0005, 0.1]  # PID gains
LANDING_THRESHOLD = 1500  # Area threshold for landing
DISTANCE_THRESHOLD = 20  # Distance threshold for moving forward

class TelloApp:
    def __init__(self, master):
        self.master = master
        master.title("Tello Drone Control")

        self.tello = Tello()
        self.tello.connect()

        # Start video stream
        try:
            self.tello.streamon()
            print("Video stream started.")
        except Exception as e:
            print(f"Failed to start video stream: {e}")
            return  # Exit if the stream cannot be started
        self.frame_read = self.tello.get_frame_read()  # Get the frame read object

        # ArUco marker setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_tracking = False
        self.pError = 0

        # Load background image
        self.background_image = Image.open("Images/Gold-Brayer2.png")  # Replace with your image file
        self.background_image = self.background_image.resize((800, 800), Image.ANTIALIAS)  # Resize to fit the window
        self.background_photo = ImageTk.PhotoImage(self.background_image)

        # Create a canvas to hold the background image
        self.canvas = TK.Canvas(master, width=800, height=800, highlightcolor='gold')
        self.canvas.pack(fill="both", expand=True)
        self.canvas.create_image(0, 0, image=self.background_photo, anchor="nw")

        # Create a frame for the controls
        self.control_frame = TK.Frame(master, bd=1)
        self.control_frame.place(relx=0.5, rely=0.85, anchor="center")

        # Video stream placeholder
        self.video_frame = TK.Frame(master, width=640, height=480, highlightbackground='yellow')
        self.video_frame.place(relx=0.5, rely=0.45, anchor="center")
        self.video_label = TK.Label(self.video_frame)
        self.video_label.pack(expand=True, fill=TK.BOTH)

        button_options = {'padx': 8, 'pady': 8, 'width': 10}

        # Add buttons for vertical movement
        self.up_button = TK.Button(self.control_frame, text="Up", command=self.move_up, **button_options)
        self.up_button.grid(row=0, column=1)

        self.down_button = TK.Button(self.control_frame, text="Down", command=self.move_down, **button_options)
        self.down_button.grid(row=2, column=1)

        # Add buttons for horizontal movement
        self.left_button = TK.Button(self.control_frame, text="Left", command=self.move_left, **button_options)
        self.left_button.grid(row=1, column=4)

        self.right_button = TK.Button(self.control_frame, text="Right", command=self.move_right, **button_options)
        self.right_button.grid(row=1, column=6)

        # Add buttons for forward and backward movement
        self.forward_button = TK.Button(self.control_frame, text="Forward", command=self.move_forward, **button_options)
        self.forward_button.grid(row=0, column=5)

        self.backward_button = TK.Button(self.control_frame, text="Backward", command=self.move_backward, **button_options)
        self.backward_button.grid(row=2, column=5)

        # Add new buttons for rotation
        self.counter_clockwise_button = TK.Button(self.control_frame, text="Spin Left", command=self.rotate_counter_clockwise, **button_options)
        self.counter_clockwise_button.grid(row=1, column=0)

        self.clockwise_button = TK.Button(self.control_frame, text="Spin Right", command=self.rotate_clockwise, **button_options)
        self.clockwise_button.grid(row=1, column=2)

        # Add buttons for takeoff, landing, and "Go To" button controls
        self.takeoff_button = TK.Button(self.control_frame, text="Take Off", highlightbackground='green3', command=self.take_off, **button_options)
        self.takeoff_button.grid(row=0, column=3)

        self.land_button = TK.Button(self.control_frame, text="Land", highlightbackground='yellow', command=self.land, **button_options)
        self.land_button.grid(row=1, column=3)

        self.emergency_stop_button = TK.Button(self.control_frame, text="Emergency Stop", highlightbackground='red3', command=self.emergency_stop, **button_options)
        self.emergency_stop_button.grid(row=3, column=3)


        self.go_to_button = TK.Button(self.control_frame, text="Go to ArUco and Land", highlightbackground='lightblue', command=self.go_to_aruco_and_land, **button_options)
        self.go_to_button.grid(row=2, column=3)

        self.camera_down = False  # Initialize camera direction

        # Battery status label
        self.battery_label = TK.Label(master, text="Battery: 100%", bg="white")
        self.battery_label.place(relx=0.5, rely=0.05, anchor="center")  # Center the battery label at the top

        # Camera switch button
        self.camera_button = TK.Button(master, text="Switch Camera", highlightbackground="gold", command=self.set_camera_direction, **button_options)
        self.camera_button.place(relx=0.5, rely=0.1, anchor="center")  # Center the camera button below the battery label

        # Start updating the video feed
        self.update_video()
        self.update_battery_status()

    def update_video(self):
        """Update the video stream from the drone."""
        img = self.frame_read.frame
        if img is not None:
            aruco_info, area, img_with_markers = self.find_aruco(img)

            # Convert the image to RGB format for Tkinter
            img_with_markers = cv2.cvtColor(img_with_markers, cv2.COLOR_BGR2RGB)
            img_with_markers = Image.fromarray(img_with_markers)
            img_with_markers = img_with_markers.resize((640, 480), Image.ANTIALIAS)
            imgtk = ImageTk.PhotoImage(img_with_markers)

            # Update the Tkinter label with the new image
            self.video_label.config(image=imgtk)
            self.video_label.image = imgtk  # Keep a reference to avoid garbage collection

            # PID control for tracking
            if aruco_info[0] != 0:
                width = img.shape[1]
                yaw_velocity, self.pError = self.track_aruco(aruco_info, width, PID, self.pError)
                if yaw_velocity != 0:
                    self.tello.rotate_clockwise(yaw_velocity)  # Rotate the drone

        self.video_label.after(10, self.update_video)  # Update every 10 ms

    def find_aruco(self, img):
        if img is None or img.size == 0:
            print("Warning: Received an empty frame in find_aruco.")
            return [0, 0], 0, np.zeros((480, 640, 3), dtype=np.uint8)

        if not self.aruco_tracking:
            return [0, 0], 0, img  # Return default values when not tracking

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(img, corners, ids)
            c = corners[0][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            area = cv2.contourArea(c.astype(np.float32))
            cv2.circle(img, (cx, cy), 5, (0, 255, 255), cv2.FILLED)
            return [cx, cy], area, img
        else:
            return [0, 0], 0, img  # No marker detected

    def track_aruco(self, info, width, pid, p_error):
        if not self.aruco_tracking or info[0] == 0:
            return 0, p_error  # Don't track if disabled or no marker

        error = info[0] - width // 2
        p = pid[0] * error
        i = pid[1] * (error + p_error)
        d = pid[2] * (error - p_error)
        yaw_velocity = int(p + i + d)
        yaw_velocity = max(min(yaw_velocity, SPEED), -SPEED)

        return yaw_velocity, error

    def go_to_aruco_and_land(self):
        self.aruco_tracking = True  # Enable ArUco tracking
        last_seen_time = time.time()
        landed = False

        while not landed:
            try:
                img = self.frame_read.frame
                if img is None or img.size == 0:
                    print("Warning: Empty frame received from drone. Skipping...")
                    continue

                frame = cv2.resize(img, (640, 480))
                aruco_info, area, processed_frame = self.find_aruco(frame)

                if aruco_info[0] != 0:
                    last_seen_time = time.time()
                    cx, cy = aruco_info
                    error_x = cx - frame.shape[1] // 2
                    error_y = cy - frame.shape[0] // 2

                    yaw_velocity, self.pError = self.track_aruco(aruco_info, frame.shape[1], PID, self.pError)
                    forward_speed = max(20, int(SPEED * (1 - min(area / (LANDING_THRESHOLD * 1.5), 1))))
                    vertical_speed = int(PID[0] * error_y)

                    if abs(error_x) > DISTANCE_THRESHOLD:
                        self.tello.send_rc_control(0, forward_speed, -vertical_speed, yaw_velocity)
                        time.sleep(1)
                    elif abs(error_x) < DISTANCE_THRESHOLD:
                        self.tello.send_rc_control(0, forward_speed, -vertical_speed, yaw_velocity)
                        time.sleep(1)

                    if area > LANDING_THRESHOLD:
                        print("Marker detected and close enough. Landing!")
                        self.tello.land()
                        landed = True
                        break
                else:
                    if time.time() - last_seen_time > 2:  # Marker lost for 2 seconds
                        print("Marker lost - hovering")
                        self.tello.send_rc_control(0, 0, 0, 0)  # Hover in place
                        time.sleep(1)

                processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                img_with_markers = Image.fromarray(processed_frame)
                img_with_markers = img_with_markers.resize((640, 480), Image.ANTIALIAS)
                imgtk = ImageTk.PhotoImage(img_with_markers)

                self.video_label.config(image=imgtk)
                self.video_label.image = imgtk  # Keep a reference to avoid garbage collection

            except Exception as e:
                print(f"Error during tracking: {e}")
                self.tello.land()
                break

        self.pError = 0
        self.aruco_tracking = False

    def set_camera_direction(self):
        if self.camera_down:
            self.camera_down = False
            self.tello.set_video_direction(self.tello.CAMERA_FORWARD)
        else:
            self.camera_down = True
            self.tello.set_video_direction(self.tello.CAMERA_DOWNWARD)

    def take_off(self):
        if not self.tello.is_connected():
            print("Error: Tello is not connected.")
            return

        try:
            self.tello.takeoff()
            print("Takeoff successful!")
        except Exception as e:
            print(f"Takeoff failed: {e}")

    def land(self):
        self.tello.land()

    def move_forward(self):
        self.tello.move_forward(30)  # Move forward 30 cm

    def move_up(self):
        self.tello.move_up(30)  # Move up 30 cm

    def move_down(self):
        self.tello.move_down(30)  # Move down 30 cm

    def move_backward(self):
        self.tello.move_back(30)  # Move backward 30 cm

    def move_left(self):
        self.tello.move_left(30)  # Move left 30 cm

    def move_right(self):
        self.tello.move_right(30)  # Move right 30 cm

    def rotate_clockwise(self):
        self.tello.rotate_clockwise(45)  # Rotate clockwise 45 degrees

    def rotate_counter_clockwise(self):
        self.tello.rotate_counter_clockwise(45)  # Rotate counter-clockwise 45 degrees

    def emergency_stop(self):
        self.tello.emergency()  # Use emergency stop

    def update_battery_status(self):
        battery_percentage = self.tello.get_battery()
        self.battery_label.config(text=f"Battery: {battery_percentage}%")
        self.master.after(1000, self.update_battery_status)  # Update every second

    def run_app(self):
        try:
            # self.update_video()  # Start the video feed update
            self.master.mainloop()  # Start the tkinter main loop
            self.update_video()
        except Exception as e:
            print(f"Error running the application: {e}")
        finally:
            self.cleanup()  # Ensure cleanup on exit

    def cleanup(self) -> None:
        try:
            print("Cleaning up resources...")
            if self.camera_down:
                self.tello.set_video_direction(self.tello.CAMERA_FORWARD)

            self.tello.end()
            self.master.quit()  # Quit the Tkinter main loop
            exit()
        except Exception as e:
            print(f"Error performing cleanup: {e}")

if __name__ == "__main__":
    root = TK.Tk()
    app = TelloApp(root)  # Pass the root window to TelloApp
    app.run_app()
