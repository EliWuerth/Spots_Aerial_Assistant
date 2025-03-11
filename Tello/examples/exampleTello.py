import tkinter as tk
from tkinter import *
from tkinter import messagebox
import random
from tkinter import simpledialog
from PIL import Image, ImageTk

class TelloDroneGUISimulator:
    def __init__(self, master):
        self.master = master
        master.title("Tello Drone Control Simulator")

        # Load background image
        self.background_image = Image.open("Images/Gold-Brayer2.png")  # Replace with your image file
        self.background_image = self.background_image.resize((800, 800), Image.ANTIALIAS)  # Resize to fit the window
        self.background_photo = ImageTk.PhotoImage(self.background_image)

        # Create a canvas to hold the background image
        self.canvas = tk.Canvas(master, width=800, height=650, highlightcolor='gold')
        self.canvas.pack(fill="both", expand=True)

        # Set the background image on the canvas
        self.canvas.create_image(0, 0, image=self.background_photo, anchor="nw")

        # Create a frame for the controls
        self.control_frame = tk.Frame(master, bd=1)
        self.control_frame.place(relx=0.5, rely=0.8, anchor="center")  # Center the control frame

        # Video stream placeholder
        self.video_frame = tk.Frame(master, width=800, height=650, highlightbackground='yellow')
        self.video_frame.place(relx=0.5, rely=0.5, anchor="center")  # Center the video frame

        button_options = {'padx': 8, 'pady': 8, 'width': 10}

        # Placeholder for video stream
        self.video_label = tk.Label(self.video_frame, text="Video Stream Placeholder", fg="black", font=("Arial", 24))
        self.video_label.pack(expand=True, fill=tk.BOTH)

        # Add buttons for vertical movement
        self.up_button = tk.Button(self.control_frame, text="Up", command=self.move_up, **button_options)
        self.up_button.grid(row=0, column=1)

        self.down_button = tk.Button(self.control_frame, text="Down", command=self.move_down, **button_options)
        self.down_button.grid(row=2, column=1)

        # Add buttons for horizontal movement
        self.left_button = tk.Button(self.control_frame, text="Left", command=self.move_left, **button_options)
        self.left_button.grid(row=1, column=4)

        self.right_button = tk.Button(self.control_frame, text="Right", command=self.move_right, **button_options)
        self.right_button.grid(row=1, column=6)

        # Add buttons for forward and backward movement
        self.forward_button = tk.Button(self.control_frame, text="Forward", command=self.move_forward, **button_options)
        self.forward_button.grid(row=0, column=5)

        self.backward_button = tk.Button(self.control_frame, text="Backward", command=self.move_backward, **button_options)
        self.backward_button.grid(row=2, column=5)

        # Add buttons for takeoff, landing, and other controls
        self.takeoff_button = tk.Button(self.control_frame, text="Take Off",  highlightbackground='green3', command=self.take_off, **button_options)
        self.takeoff_button.grid(row=0, column=3)

        self.land_button = tk.Button(self.control_frame, text="Land", highlightbackground='yellow', command=self.land, **button_options)
        self.land_button.grid(row=1, column=3)

        # Add new buttons for rotation and emergency stop
        self.clockwise_button = tk.Button(self.control_frame, text="Spin Right", command=self.rotate_clockwise, **button_options)
        self.clockwise_button.grid(row=1, column=2)

        self.counter_clockwise_button = tk.Button(self.control_frame, text="Spin Left", command=self.rotate_counter_clockwise, **button_options)
        self.counter_clockwise_button.grid(row=1, column=0)

        self.emergency_stop_button = tk.Button(self.control_frame, text="Emergency Stop", highlightbackground='red3', command=self.emergency_stop, **button_options)
        self.emergency_stop_button.grid(row=3, column=3)  # Span across three columns

        # Add "Go To" button
        self.go_to_button = tk.Button(self.control_frame, text="Go To", highlightbackground='medium purple', command=self.go_to, **button_options)
        self.go_to_button.grid(row=2, column=3)  # Span across three columns

        # Battery status label
        self.battery_label = tk.Label(master, text="Battery: 100%", bg="white")
        self.battery_label.place(relx=0.5, rely=0.05, anchor="center")  # Center the battery label at the top

        # Camera switch button
        self.camera_button = tk.Button(master, text="Switch Camera", highlightbackground="gold", command=self.switch_camera, **button_options)
        self.camera_button.place(relx=0.5, rely=0.1, anchor="center")  # Center the camera button below the battery label

        # Simulate battery status updates
        self.update_battery_status()
        
    def go_to(self):
        location = simpledialog.askstring("Go To", "Enter the location (e.g., x,y):")
        if location:
            self.update_status(f"Drone moving to {location}...")
        
    def rotate_clockwise(self):
        self.update_status("Drone rotating clockwise...")

    def rotate_counter_clockwise(self):
        self.update_status("Drone rotating counter-clockwise...")

    def emergency_stop(self):
        self.update_status("Emergency stop activated!")

    def take_off(self):
        self.update_status("Drone taking off...")

    def land(self):
        self.update_status("Drone landing...")

    def move_forward(self):
        self.update_status("Drone moving forward...")

    def move_backward(self):
        self.update_status("Drone moving backward...")

    def move_left(self):
        self.update_status("Drone moving left...")

    def move_right(self):
        self.update_status("Drone moving right...")

    def move_up(self):
        self.update_status("Drone moving up...")

    def move_down(self):
        self.update_status("Drone moving down...")

    def switch_camera(self):
        self.update_status("Camera switched.")

    def update_status(self, message):
        messagebox.showinfo("Status", message)

    def update_battery_status(self):
        # Simulate battery percentage
        battery_percentage = random.randint(0, 100)
        self.battery_label.config(text=f"Battery: {battery_percentage}%")
        self.master.after(5000, self.update_battery_status)  # Update every 5 seconds

if __name__ == "__main__":
    root = tk.Tk()
    app = TelloDroneGUISimulator(root)
    root.mainloop()