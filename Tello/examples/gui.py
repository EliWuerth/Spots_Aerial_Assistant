import cv2
import yaml
import numpy as np
from djitellopy import tello
import tkinter as TK
from PIL import Image, ImageTk

class TelloApp:
    def __init__(self):
        self.root = TK.Tk()
        self.root.title("Tello Drone Control")

        self.tello = tello.Tello()
        self.tello.connect()
        print("Battery percentage:", self.tello.get_battery())
        self.tello.set_speed(50)

        self.tello.streamon()

        self.frame_read = self.tello.get_frame_read()

        #sets camera forward
        self.tello.CAMERA_FORWARD
        self.camera_down = False

        # Load camera calibration data
        # self.camera_matrix, self.dist_coeffs = self.load_camera_calibration('calibration_matrix.yaml')

        # Create a canvas for video feed
        self.canvas = TK.Canvas(self.root, width=640, height=480, bg='black')
        self.canvas.grid(column=0, row=0, columnspan=6, padx=10, pady=10)

        # Create control buttons with uniform size and padding
        button_options = {'padx': 10, 'pady': 10, 'width': 12}

        # Column 1
        self.takeoff_button = TK.Button(self.root, text="Take Off", command=self.takeoff, **button_options)
        self.takeoff_button.grid(column=0, row=1)

        self.land_button = TK.Button(self.root, text="Land", command=self.land, **button_options)
        self.land_button.grid(column=0, row=2)

        self.camera_dir_button = TK.Button(self.root, text="Switch Camera", command=self.set_camera_direction, **button_options)
        self.camera_dir_button.grid(column=0, row=5)

        # Column 2

        self.off_button = TK.Button(self.root, text="Emergency Stop", command=self.turn_off, **button_options)
        self.off_button.grid(column=1, row=5)

        # Column 3
        self.forward_button = TK.Button(self.root, text="Forward", command=self.move_forward, **button_options)
        self.forward_button.grid(column=2, row=3)

        self.backward_button = TK.Button(self.root, text="Backward", command=self.move_backward, **button_options)
        self.backward_button.grid(column=2, row=4)

        # Column 4
        self.left_button = TK.Button(self.root, text="Left", command=self.move_left, **button_options)
        self.left_button.grid(column=3, row=2)
        
        self.clockwise_button = TK.Button(self.root, text="Clockwise", command=self.clockwise, **button_options)
        self.clockwise_button.grid(column=3, row=1)

        # Column 5
        self.up_button = TK.Button(self.root, text="Up", command=self.move_up, **button_options)
        self.up_button.grid(column=4, row=1)

        self.go_button = TK.Button(self.root, text="Go", command=self.go, **button_options)
        self.go_button.grid(column=4, row=2)
        
        self.down_button = TK.Button(self.root, text="Down", command=self.move_down, **button_options)
        self.down_button.grid(column=4, row=3)

        # Column 6
        self.counter_clockwise_button = TK.Button(self.root, text="Counter-Clockwise", command=self.counter_clockwise, **button_options)
        self.counter_clockwise_button.grid(column=5, row=1)
        
        self.right_button = TK.Button(self.root, text="Right", command=self.move_right, **button_options)
        self.right_button.grid(column=5, row=2)

        # Adjust grid weights for better resizing behavior
        for i in range(6):
            self.root.grid_columnconfigure(i, weight=1)

    # def load_camera_calibration(self, file_path):
    #     """ Load camera calibration data from YAML file. """
    #     with open(file_path, 'r') as file:
    #         data = yaml.safe_load(file)
    #     return np.array(data['camera_matrix']), np.array(data['dist_coeff'][0])

    #makes a camera toggle, so it can switch between the cameras
    def set_camera_direction(self):
        if self.camera_down:
            self.camera_down = False
            self.tello.set_video_direction(self.tello.CAMERA_FORWARD)
            print(self.tello.CAMERA_FORWARD)
        else:
            self.camera_down = True
            self.tello.set_video_direction(self.tello.CAMERA_DOWNWARD)
            print(self.tello.CAMERA_DOWNWARD)

    
    def takeoff(self):
        self.tello.takeoff()

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
    
    def clockwise(self):
        self.tello.rotate_clockwise(45)  # Rotate clockwise 45 degrees

    def counter_clockwise(self):
        self.tello.rotate_counter_clockwise(45)  # Rotate counter-clockwise 45 degrees

    def go(self):
        self.tello.go_xyz_speed(50, 50, 50, 100)  # Example movement

    def turn_off(self):
        self.tello.emergency()  # Use emergency stop

    def update_video(self):
        img = self.frame_read.frame
        
        #resize and rotate the camera if we are using the down camera
        if self.camera_down:
            img = cv2.resize(img, [640, 480])
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

        # **Apply Undistortion**
        # undistorted_img = cv2.undistort(img, self.camera_matrix, self.dist_coeffs)

        # Convert for Tkinter
        # new_frame = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2RGB)
        new_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        frames = cv2.cvtColor(new_frame, cv2.COLOR_RGB2BGR)
        newimg = Image.fromarray(frames)
        imgtk = ImageTk.PhotoImage(image=newimg)
        self.canvas.create_image(0, 0, anchor=TK.NW, image=imgtk)
        self.canvas.imgtk = imgtk  # Keep a reference to avoid garbage collection
        self.canvas.after(5, self.update_video)  # Update the video feed every 5 ms
    
    def run_app(self):
        try:
            self.update_video()  # Start the video feed update
            self.root.mainloop()  # Start the tkinter main loop
        except Exception as e:
            print(f"Error running the application: {e}")
        finally:
            self.cleanup()  # Ensure cleanup on exit

    def cleanup(self) -> None:
        try:
            print("Cleaning up resources...")

            #reset the camrea to forward
            if self.camera_down:
                self.tello.set_video_direction(self.tello.CAMERA_FORWARD)

            self.tello.end()
            self.root.quit()  # Quit the Tkinter main loop
            exit()
        except Exception as e:
            print(f"Error performing cleanup: {e}")

if __name__ == "__main__":
    app = TelloApp()
    app.run_app()
