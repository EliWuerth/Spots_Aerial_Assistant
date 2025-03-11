import cv2
import numpy as np
from djitellopy import tello
import tkinter as TK
from PIL import Image, ImageTk

class TelloApp:
    def __init__(self, master):
        self.master = master
        # self.root = TK.Tk()  # Removed second Tk instance
        master.title("Tello Drone Control")

        self.tello = tello.Tello()
        self.tello.connect()
        print("Battery percentage:", self.tello.get_battery())
        self.tello.set_speed(50)

        self.tello.streamon()

        # Load background image
        self.background_image = Image.open("Images/Gold-Brayer2.png")  # Replace with your image file
        self.background_image = self.background_image.resize((800, 800), Image.ANTIALIAS)  # Resize to fit the window
        self.background_photo = ImageTk.PhotoImage(self.background_image)

        # Create a canvas to hold the background image
        self.canvas = TK.Canvas(master, width=800, height=800, highlightcolor='gold')
        self.canvas.pack(fill="both", expand=True)

        # Set the background image on the canvas
        self.canvas.create_image(0, 0, image=self.background_photo, anchor="nw")

        # Create a frame for the controls
        self.control_frame = TK.Frame(master, bd=1)
        self.control_frame.place(relx=0.5, rely=0.85, anchor="center")

        self.frame_read = self.tello.get_frame_read()

        # Video stream placeholder
        self.video_frame = TK.Frame(master, width=640, height=480, highlightbackground='yellow')
        self.video_frame.place(relx=0.5, rely=0.45, anchor="center")

        self.video_label = TK.Label(self.video_frame, fg="black", font=("Arial", 24))
        self.video_label.pack(expand=True, fill=TK.BOTH)

        button_options = {'padx': 8, 'pady': 8, 'width': 10}
        
        #sets camera forward
        self.tello.CAMERA_FORWARD
        self.camera_down = False

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
        self.takeoff_button = TK.Button(self.control_frame, text="Take Off",  highlightbackground='green3', command=self.take_off, **button_options)
        self.takeoff_button.grid(row=0, column=3)

        self.land_button = TK.Button(self.control_frame, text="Land", highlightbackground='yellow', command=self.land, **button_options)
        self.land_button.grid(row=1, column=3)

        self.emergency_stop_button = TK.Button(self.control_frame, text="Emergency Stop", highlightbackground='red3', command=self.emergency_stop, **button_options)
        self.emergency_stop_button.grid(row=3, column=3)

        self.go_to_button = TK.Button(self.control_frame, text="Go To", highlightbackground='medium purple', command=self.go_to, **button_options)
        self.go_to_button.grid(row=2, column=3)

        # Battery status label
        self.battery_label = TK.Label(master, text="Battery: 100%", bg="white")
        self.battery_label.place(relx=0.5, rely=0.05, anchor="center")  # Center the battery label at the top

        # Camera switch button
        self.camera_button = TK.Button(master, text="Switch Camera", highlightbackground="gold", command=self.set_camera_direction, **button_options)
        self.camera_button.place(relx=0.5, rely=0.1, anchor="center")  # Center the camera button below the battery label

        # Simulate battery status updates
        self.update_battery_status()

    #makes a camera toggle, so it can switch between the cameras
    def set_camera_direction(self):
        if self.camera_down:
            self.camera_down = False
            self.tello.set_video_direction(self.tello.CAMERA_FORWARD)
            # print(self.tello.CAMERA_FORWARD)
        else:
            self.camera_down = True
            self.tello.set_video_direction(self.tello.CAMERA_DOWNWARD)
            # print(self.tello.CAMERA_DOWNWARD)

    
    def take_off(self):
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
    
    def rotate_clockwise(self):
        self.tello.rotate_clockwise(45)  # Rotate clockwise 45 degrees

    def rotate_counter_clockwise(self):
        self.tello.rotate_counter_clockwise(45)  # Rotate counter-clockwise 45 degrees

    def go_to(self):
        self.tello.go_xyz_speed(50, 50, 50, 100)  # Example movement

    def emergency_stop(self):
        self.tello.emergency()  # Use emergency stop

    def update_battery_status(self):
        # Send command to get battery status
        battery_percentage = self.tello.get_battery()  # Assuming get_battery() is the correct method
        self.battery_label.config(text=f"Battery: {battery_percentage}%")
        self.master.after(1000, self.update_battery_status)  # Update every 5 seconds

    def update_video(self):
        if self.frame_read is None:
            print("Frame read is not initialized.")
            return
        
        img = self.frame_read.frame
        
        # Resize and rotate the camera if we are using the down camera
        if self.camera_down:
            img = cv2.resize(img, [640, 480])
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

        new_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        frames = cv2.cvtColor(new_frame, cv2.COLOR_RGB2BGR)
        newimg = Image.fromarray(frames)
        frame_image = newimg.resize((640, 480), Image.ANTIALIAS)
        imgtk = ImageTk.PhotoImage(frame_image)
        self.video_label.config(image=imgtk)
        self.video_label.image = imgtk  # Keep a reference to avoid garbage collection
        self.video_label.after(10, self.update_video)

    def run_app(self):
        try:
            self.update_video()  # Start the video feed update
            self.master.mainloop()  # Start the tkinter main loop
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
            self.master.quit()  # Quit the Tkinter main loop
            exit()
        except Exception as e:
            print(f"Error performing cleanup: {e}")

if __name__ == "__main__":
    root = TK.Tk()
    app = TelloApp(root)  # Pass the root window to TelloApp
    app.run_app()
