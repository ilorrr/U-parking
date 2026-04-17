# ============================================================
# demo_display.py
# Conference demo display using Quanser's Probe/Observer system.
# Shows live drone camera feed and real-time occupancy stats
# on a remote display — perfect for projecting during a talk.
#
# Two components:
#   1. DemoSender   — runs alongside the scan, sends frames + stats
#   2. DemoReceiver — runs on the display machine, shows everything
#
# Usage:
#   Display machine:  python demo_display.py --receive
#   Scan machine:     python demo_display.py --send
#
# Or integrate DemoSender into main.py scan loop.
# ============================================================

import sys
import time
import cv2
import numpy as np



import sys
sys.path.insert(0, r"C:\Users\Roli\Documents\uparking")

try:
    from pal.utilities.probe import Probe, Observer
    PROBE_AVAILABLE = True
except ImportError:
    PROBE_AVAILABLE = False
    print("[DemoDisplay] WARNING: pal.utilities.probe not available")


# ============================================================
# CONFIGURATION
# ============================================================

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CHANNELS = 3
DISPLAY_IP = "localhost"  # change for remote display


# ============================================================
# SENDER — runs on the scan machine alongside main.py
# ============================================================

class DemoSender:
    """
    Sends drone frames and occupancy data to the demo display.

    Usage in main.py:
        sender = DemoSender()
        sender.connect()

        # In scan loop:
        sender.send_frame(frame)
        sender.send_occupancy(timestamp, occupied_count, empty_count)

        sender.stop()
    """

    def __init__(self, ip=DISPLAY_IP):
        if not PROBE_AVAILABLE:
            raise RuntimeError("Probe not available")

        self.probe = Probe(ip=ip)

        # Add displays and scopes
        self.probe.add_display(
            imageSize=[IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS],
            scalingFactor=2,
            name='Drone Camera'
        )
        self.probe.add_scope(
            numSignals=2,
            name='Occupancy'
        )

        self.connected = False

    def connect(self, timeout=30):
        """Wait for the receiver to connect."""
        print(f"[DemoSender] Waiting for display to connect...")
        start = time.time()
        while (time.time() - start) < timeout:
            if not self.probe.connected:
                self.probe.check_connection()
            if self.probe.connected:
                print("[DemoSender] Display connected!")
                self.connected = True
                return True
            time.sleep(0.1)
        print("[DemoSender] WARNING: Display not connected")
        return False

    def send_frame(self, frame):
        """Send a camera frame to the display."""
        if not self.connected:
            return False
        try:
            return self.probe.send('Drone Camera', imageData=frame)
        except Exception:
            return False

    def send_occupancy(self, timestamp, occupied, empty):
        """Send occupancy counts to the live graph."""
        if not self.connected:
            return False
        try:
            data = np.array([occupied, empty], dtype=np.float64)
            return self.probe.send('Occupancy',
                                    scopeData=(timestamp, data))
        except Exception:
            return False

    def stop(self):
        """Terminate the probe."""
        if self.probe is not None:
            self.probe.terminate()
        print("[DemoSender] Stopped")


# ============================================================
# RECEIVER — runs on the display machine (projector/screen)
# ============================================================

class DemoReceiver:
    """
    Receives and displays drone frames and occupancy data.
    Run this on the presentation laptop connected to the projector.

    Usage:
        receiver = DemoReceiver()
        receiver.launch()  # blocks until interrupted
    """

    def __init__(self):
        if not PROBE_AVAILABLE:
            raise RuntimeError("Observer not available")

        self.observer = Observer()

        # Add matching displays and scopes
        self.observer.add_display(
            imageSize=[IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS],
            scalingFactor=2,
            name='Drone Camera'
        )
        self.observer.add_scope(
            numSignals=2,
            name='Occupancy'
        )

    def launch(self):
        """
        Launch the display — blocks until window is closed
        or user presses Ctrl+C.
        """
        print("[DemoReceiver] Launching display...")
        print("  - Drone Camera: live downward view")
        print("  - Occupancy: real-time occupied/empty counts")
        print("  Press Ctrl+C to stop")
        self.observer.launch()


# ============================================================
# CALLBACK FOR SCAN LOOP INTEGRATION
# ============================================================

def make_demo_callback(sender):
    """
    Returns a callback function for use with LiveScanner or
    vision_collection_pass.

    Usage:
        sender = DemoSender()
        sender.connect()
        callback = make_demo_callback(sender)

        # In scan loop at each waypoint:
        callback(label, frame, occupied, scan_stats)
    """
    start_time = time.time()
    stats = {"occupied": 0, "empty": 0}

    def callback(label, frame, occupied, _=None):
        if occupied:
            stats["occupied"] += 1
        else:
            stats["empty"] += 1

        timestamp = time.time() - start_time

        # Send frame
        sender.send_frame(frame)

        # Send running totals
        sender.send_occupancy(
            timestamp,
            stats["occupied"],
            stats["empty"]
        )

    return callback


# ============================================================
# STANDALONE EXECUTION
# ============================================================

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage:")
        print("  Display machine:  python demo_display.py --receive")
        print("  Scan machine:     python demo_display.py --send")
        sys.exit(1)

    mode = sys.argv[1]

    if mode == "--receive":
        receiver = DemoReceiver()
        receiver.launch()

    elif mode == "--send":
        # Demo mode: send test pattern
        sender = DemoSender()
        try:
            if sender.connect():
                print("[Demo] Sending test frames...")
                for i in range(300):
                    # Create test frame with color
                    frame = np.random.randint(
                        0, 255,
                        (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS),
                        dtype=np.uint8
                    )
                    cv2.putText(frame, f"Frame {i}",
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1.0, (255, 255, 255), 2)

                    sender.send_frame(frame)
                    sender.send_occupancy(
                        i / 30.0,
                        np.random.randint(20, 80),
                        np.random.randint(200, 300)
                    )
                    time.sleep(0.033)
            else:
                print("[Demo] No display connected — run --receive first")
        finally:
            sender.stop()

    else:
        print(f"Unknown mode: {mode}")
        print("Use --receive or --send")
