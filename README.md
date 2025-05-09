# Spots Aerial Assistant

## Project Description
Our capstone project focuses on developing a drone-assisted system for SPOT, utilizing the DJI Tello drone. This proof-of-concept integrates autonomous navigation, computer vision, and custom-built GUI tools to demonstrate how robotics can support real-world emergency scenarios.

### Tello GUI
We built a Python-based GUI for the Tello that enables both manual and autonomous control. It incorporates ArUco marker tracking for precision navigation to and from SPOT, which acts as a mobile base. The interface includes features such as live video feed, camera switching, battery monitoring, and custom functions like human tracking and "Go to ArUco."

### SPOT GUI
In addition to the drone interface, we developed a dedicated GUI for controlling and monitoring SPOT. Built using Python and the Spot SDK, the GUI provides real-time feedback on Spot’s battery, temperature, and system health. It supports keyboard-based manual navigation as well as automated posture and docking commands.

This project showcases the potential for low-cost, scalable solutions in search and rescue operations and provided us hands-on experience with robotics, drone programming, and computer vision.

Below you will find basic control and set up instructions.

## Clone the Repository
```
git clone https://github.com/EliWuerth/Spots_Aerial_Assistant.git
```

---

## Python Installation
This project requires **Python 3.9.8** (published on Nov. 5, 2021).  
Download and install it from:  
🔗 [Python 3.9.8 Downloads](https://www.python.org/downloads/)

## Install Virtual Environment (`venv`)
 **MacOS**
```
python3 -m pip install venv
```

 **Windows**
```
py.exe -3.9 -m pip install venv
```

---

## Setup the Spot Virtual Environment

1. **Change into the Spot directory:**
   ```
   cd Spot
   ```

2. **Create the virtual environment:**
   -  **MacOS**  
     ```
     python3 -m venv spot_env
     ```
   -  **Windows**  
     ```
     py.exe -3.9 -m venv spot_env
     ```

3. **Activate the virtual environment:**
   -  **MacOS**  
     ```
     source spot_env/bin/activate
     ```
   -  **Windows**  
     ```
     .\spot_env\Scripts\activate.bat
     ```
4. **Upgrade Pip**
   - **MacOS**
   ```
   python3.exe -m pip install --upgrade pip
   ```
   - **Windows**
   ```
   python.exe -m pip install --upgrade pip
   ```

5. **Install BosDyn dependencies:**
   ```
   python -m pip install bosdyn-client==4.0.3 bosdyn-mission==4.0.3 bosdyn-choreography-client==4.0.3 bosdyn-orbit==4.0.3
   ```

6. **Clone the Boston Dynamics repository and install requirements:**
   ```
   git clone https://github.com/boston-dynamics/spot-sdk.git
   python -m pip install -r requirements.txt
   ```

7. **To exit the virtual environment:**
   ```
   deactivate
   ```
---

## Running the SPOT Program
**Connect to SPOT**  
   - Turn on **SPOT**  
   - Connect your device to the **WiFi network** (e.g., `"spot-BD-########"`)
 **MacOS**
```
cd examples/gui
python3 gui.py
```

 **Windows**
```
cd examples/gui
python gui.py
```
**Log in or register**
```
Create a username and password
and log in
```
Then you use SPOT's Creditials to log in

To then operate SPOT you must 'Take lease'
---

## Setup the Tello Virtual Environment

1. **Change into the Tello directory:**
   ```
   cd ..
   cd Tello
   ```

2. **Create the Tello virtual environment:**
   -  **MacOS**  
     ```
     python3 -m venv tello_env
     ```
   -  **Windows**  
     ```
     py.exe -3.9 -m venv tello_env
     ```

3. **Activate the virtual environment:**
   -  **MacOS**
   ```
   source tello_env/bin/activate
   ```
   -  **Windows**
     ```
     .\tello_env\Scripts\activate.bat
     ```

4. **Upgrade Pip**
   -  **MacOS**
   ```
   python3.exe -m pip install --upgrade pip
   ```
   -  **Windows**
   ```
   python.exe -m pip install --upgrade pip
   ```

5. **Install dependencies:**
   ```
   python -m pip install -r requirements.txt
   ```
---

## Running the Drone Program
 **Connect to the DJI Tello Drone**  
   - Turn on the **DJI Tello drone**  
   - Connect your device to the **WiFi network** (e.g., `"TELLO-######"`)
   
 **MacOS**
```
cd examples
python3 gui.py
```

 **Windows**
```
cd examples
python gui.py
```
---

### BASIC CONTROLS

**FOR TELLO**
```
W,A,S,D for basic movement
Q,E for Rotation
UP and DOWN arrow keys for vertical
```
**FOR SPOT**
W,A,S,D for basic movement
For other fucntions please refer to (`"reference_images/spot_controls.jpg"`)

**Other functions**
Use buttons and try them out and have fun `:)`