# UParking Simulation

UParking is a drone-based parking lot simulation and computer vision occupancy detection system built on top of Quanser QLabs and Unreal Engine. 

This repository contains the Python backend, drone control logic (using kinematic and velocity control), Hungarian-algorithm-based QCar routing, and a live Flask web dashboard for real-time occupancy monitoring.

---

## Prerequisites & Quanser Setup

Because this project interfaces with proprietary Quanser simulation hardware/software, you must have the Quanser SDK (or QUARC) installed locally on your machine before setting up the Python environment.

1. **Install Quanser QLabs:** Ensure QLabs and the Unreal Engine environment are fully installed and licensed on your PC.
2. **Locate your Quanser Python Libraries:** By default, Quanser installs its Python `.whl` files locally on your machine (usually in `C:\Program Files\Quanser\QUARC\python` or an equivalent SDK folder).

---

## Installation & Python Setup

It is highly recommended to run this project inside a Python Virtual Environment (`venv`) to prevent dependency conflicts.

### 1. Create and Activate Virtual Environment
Open your terminal in the root of this project and run:

**Windows (PowerShell):**
powershell


python -m venv venv
.\venv\Scripts\Activate.ps1


2. Install Standard Dependencies
With your (venv) active, install the standard open-source Python packages required for the computer vision, math, and web dashboard:

PowerShell
pip install opencv-python numpy scipy flask requests

3. Install Quanser APIs (Local Install)
Do not run standard pip install quanser-api. Quanser is not hosted on standard public package repositories. You must tell your virtual environment to install it from your local hard drive.

Run this command in PowerShell (ensure your (venv) is active):

PowerShell
pip install --find-links "$env:QSDK_DIR\python" quanser_api

Troubleshooting: If $env:QSDK_DIR is not recognized by your system, use the direct absolute path to your Quanser installation folder. For example:

PowerShell
pip install --find-links "C:\Program Files\Quanser\QUARC\python" quanser_api

Configuration: Fixing Hardcoded Paths
If you are cloning this repository to a new computer, the script will crash immediately because it is looking for Quanser libraries inside specific user folders (e.g., C:\Users\Roli\...).

You must update the absolute paths in the following files to match your local machine's directory structure.
Files to Update:

****main.py
****drone_scanner.py
****live_scanner.py
****qcar_spawner.py

Variables to Change (found at the top of the files):
1. QLABS_LIB_PATH
This needs to point to the root folder of this cloned repository.

Example: QLABS_LIB_PATH = r"C:\Users\YourName\Desktop\U-parking"

2. QUANSER_LIB_PATH
This points to the main Quanser Python library folder installed on your system.

Example: QUANSER_LIB_PATH = r"C:\Users\YourName\Documents\Quanser\0_libraries\python"

3. os.environ['RTMODELS_DIR'] (Found in main.py)
This points to the Quanser Real-Time models directory.

Example: os.environ['RTMODELS_DIR'] = r"C:\Users\YourName\Documents\Quanser\0_libraries\resources\rt_models"

Pro-Tip: Make Paths Dynamic
To avoid having to change QLABS_LIB_PATH every time the code moves to a new PC, you can replace the hardcoded string with Python's dynamic pathing:

Python
import os
import sys
# Automatically finds the folder where this script is running
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
QLABS_LIB_PATH = os.path.dirname(CURRENT_DIR) # Adjust depending on folder depth

if QLABS_LIB_PATH not in sys.path:
    sys.path.insert(0, QLABS_LIB_PATH)
🚀 Running the Simulation
Launch QLabs: Ensure the Quanser QLabs Unreal Engine window is open or allowed to be launched by the script.

Start the Script: From your active virtual environment, run the main orchestrator:

PowerShell
python simulation\main.py
View the Live Dashboard: Once the script says the server is running, open your web browser and navigate to:
http://127.0.0.1:5000

Note on Exiting:
To safely shut down the simulation and the Flask server, press Ctrl + C in the terminal. If the Python script crashes unexpectedly, it is highly recommended to close and restart the QLabs Unreal Engine window to clear the communication pipeline before running main.py again.


