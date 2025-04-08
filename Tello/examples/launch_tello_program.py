import os
import sys
from skeletongui import KeyboardVideoDemo
from PyQt5.QtWidgets import QApplication

def connect_to_drone_wifi():
    # Command to connect to the drone's Wi-Fi
    os.system("nmcli d wifi connect 'Tello-98FBB8'")  # Replace 'Tello-XXXX' with the actual SSID of the drone

if __name__ == "__main__":
    # connect_to_drone_wifi()  # Connect to the drone's Wi-Fi
    app = QApplication(sys.argv)  # Must come before any QWidget
    window = KeyboardVideoDemo()  # Now you can create your window
    window.show()
    sys.exit(app.exec_())

