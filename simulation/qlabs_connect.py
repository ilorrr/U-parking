# ============================================================
# qlabs_connect.py
# Connect to an existing QLabs instance or launch a new one.
# ============================================================

import sys
import time
import subprocess

from qvl.qlabs import QuanserInteractiveLabs

QLABS_EXE = r"C:\Program Files\Quanser\Quanser Interactive Labs\Quanser Interactive Labs.exe"


def connect_or_launch_qlabs():
    """
    Connect to QLabs on localhost.
    If no instance is running, launch the executable and retry.

    Returns:
        QuanserInteractiveLabs connection object
    """
    qlabs = QuanserInteractiveLabs()

    print("Attempting to connect to existing QLabs...")
    if qlabs.open("localhost"):
        print("Connected to existing QLabs instance")
        return qlabs

    print("No running QLabs detected. Launching QLabs...")
    subprocess.Popen([QLABS_EXE, "-loadmodule", "-Plane"])
    time.sleep(12)  # Allow QLabs to fully initialise

    print("Re-attempting connection...")
    if not qlabs.open("localhost"):
        print("Unable to connect to QLabs after launch")
        sys.exit()

    print("Connected to newly launched QLabs")
    return qlabs
