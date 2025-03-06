# Spots Aerial Assistant

## Clone the Repository
```bash
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
```bash
python3 -m pip install --upgrade pip==25.0.1
```

 **Windows**
```powershell
python -m pip install --upgrade pip==25.0.1
```

---

## Install Virtual Environment (`venv`)
 **MacOS**
```bash
python3 -m pip install venv
```

 **Windows**
```powershell
py.exe -3.9 -m pip install venv
```

---

## Setup the Spot Virtual Environment

1. **Change into the Spot directory:**
   ```bash
   cd Spot
   ```

2. **Create the virtual environment:**
   -  **MacOS**  
     ```bash
     python3 -m venv spot_env
     ```
   -  **Windows**  
     ```powershell
     py.exe -3.9 -m venv spot_env
     ```

3. **Activate the virtual environment:**
   -  **MacOS**  
     ```bash
     source spot_env/bin/activate
     ```
   -  **Windows**  
     ```powershell
     .\spot_env\Scripts\activate.bat
     ```

4. **Install BosDyn dependencies:**
   ```bash
   python -m pip install bosdyn-client==4.0.3 bosdyn-mission==4.0.3 bosdyn-choreography-client==4.0.3 bosdyn-orbit==4.0.3
   ```

5. **Clone the Boston Dynamics repository and install requirements:**
   ```bash
   git clone https://github.com/boston-dynamics/spot-sdk.git
   python -m pip install -r requirements.txt
   ```

6. **To exit the virtual environment:**
   ```bash
   deactivate
   ```

---

## Setup the Tello Virtual Environment

1. **Change into the Tello directory:**
   ```bash
   cd ..
   cd Tello
   ```

2. **Create the Tello virtual environment:**
   -  **MacOS**  
     ```bash
     python3 -m venv tello_env
     ```
   -  **Windows**  
     ```powershell
     py.exe -3.9 -m venv tello_env
     ```

3. **Activate the virtual environment:**
   -  **MacOS**  
     ```bash
     source tello_env/bin/activate
     ```
   -  **Windows**  
     ```powershell
     .\tello_env\Scripts\activate.bat
     ```

4. **Clone the DJI Tello repository:**
   ```bash
   git clone https://github.com/damiafuentes/DJITelloPy.git
   ```

5. **Install dependencies:**
   ```bash
   python -m pip install -r requirements.txt
   ```

6. **Connect to the DJI Tello Drone**  
   - Turn on the **DJI Tello drone**  
   - Connect your device to the **WiFi network** (e.g., `"TELLO-######"`)

---

## Running the Program
 **MacOS**
```bash
python3 gui.py
```

 **Windows**
```powershell
./gui.py
```
---

