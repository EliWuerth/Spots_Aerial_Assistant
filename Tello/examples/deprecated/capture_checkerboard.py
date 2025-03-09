import cv2
from djitellopy import Tello
import time

tello = Tello()
tello.connect()
print("Battery:", tello.get_battery())

tello.streamon()

# Capture images
for i in range(15):  # Take 15 images
    frame = tello.get_frame_read().frame
    filename = f'checkerboard_images/image_{i}.jpg'
    cv2.imwrite(filename, frame)
    print(f"Captured {filename}")
    time.sleep(1)  # Wait 1 second between captures

tello.streamoff()
tello.end()
print("Images captured successfully! Now run the calibration script.")
