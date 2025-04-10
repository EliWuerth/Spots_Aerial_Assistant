# Spots Aerial Assistant

## Clone the Repository
```
git clone https://github.com/EliWuerth/Spots_Aerial_Assistant.git
```

---

## Python Installation
This project requires **Python 3.9.8** (published on Nov. 5, 2021).  
Download and install it from:  
🔗 [Python 3.9.8 Downloads](https://www.python.org/downloads/)

---

## Upgrade Pip
Ensure `pip` is updated to version **25.0.1**:

 **MacOS**
```
python3 -m pip install --upgrade pip==25.0.1
```

 **Windows**
```
python -m pip install --upgrade pip==25.0.1
```

---

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

4. **Install BosDyn dependencies:**
   ```
   python -m pip install bosdyn-client==4.0.3 bosdyn-mission==4.0.3 bosdyn-choreography-client==4.0.3 bosdyn-orbit==4.0.3
   ```

5. **Clone the Boston Dynamics repository and install requirements:**
   ```
   git clone https://github.com/boston-dynamics/spot-sdk.git
   python -m pip install -r requirements.txt
   ```

6. **To exit the virtual environment:**
   ```
   deactivate
   ```

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

4. **Clone the DJI Tello repository:**
   ```
   git clone https://github.com/damiafuentes/DJITelloPy.git
   ```

5. **Install dependencies:**
   ```
   python -m pip install -r requirements.txt
   ```
---

## Running the Program
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

