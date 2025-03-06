# Spots_Aerial_Assistant

## Git clone our project
```
git clone https://github.com/EliWuerth/Spots_Aerial_Assistant.git
```
## Python Installation
This project operates on Python version 3.9.8 published on Nov. 5, 2021
Navigate to the following URL and download the correct installation
```
https://www.python.org/downloads/
```

## Pip Installation 
### MacOS
```
python3 -m pip install --upgrade pip==25.0.1
```
### Windows
```
python -m pip install --upgrade pip==25.0.1
```
## Venv Installation
### MacOS
```
python3 -m pip install venv
```
### Windows
```
py.exe -3.9 -m pip install venv
```

## Setting up your virtual enviornments

Create a new virtual environment in your desired directory
### MacOS
```
python3 -m venv spot_env
```
### Windows
```
py.exe -3.9 -m venv spot_env
```
## Activate your virtual environment for MacOS
```
source spot_env/bin/activate
```
## How to deactivate
```
deactivate
```
## In your active venv install BosDyn dependencies (MacOS)
```
python3 -m pip install bosdyn-client==4.0.3 bosdyn-mission==4.0.3 bosdyn-choreography-client==4.0.3 bosdyn-orbit==4.0.3 
```
## Clone the BosDyn repo and install requirements (MacOS)
```
cd Spot
git clone https://github.com/boston-dynamics/spot-sdk.git
python3 -m pip install -r requirements.txt
```
## Activate your virtual environment for Windows
```
.\spot_env\Scripts\activate.bat
```
## How to deactivate
```
deactivate
```
## In your active venv install BosDyn dependencies (Windows)
```
python -m pip install bosdyn-client==4.0.3 bosdyn-mission==4.0.3 bosdyn-choreography-client==4.0.3 bosdyn-orbit==4.0.3 
```
## Clone the BosDyn repo and install requirements (Windows)
```
cd Spot
git clone https://github.com/boston-dynamics/spot-sdk.git
python -m pip install -r requirements.txt
```


## Tello_Drone
```
cd Tello_Drone
git clone https://github.com/damiafuentes/DJITelloPy.git
```

