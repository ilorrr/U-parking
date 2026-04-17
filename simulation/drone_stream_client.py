# ============================================================
# drone_stream_client.py
# Runs on the QDrone2 (Jetson Xavier NX) or in QLabs simulation.
# Captures frames from the downward camera and streams them
# to the ground station server via Quanser BasicStream TCP.
#
# The ground station receives frames, runs CNN classification,
# and pushes results to the Django backend.
#
# Works in BOTH simulation and hardware — Camera2D abstracts
# the difference.
# ============================================================

import sys
import time
import cv2
import numpy as np
import json

try:
    from pal.utilities.stream import BasicStream
    from quanser.common import Timeout
except ImportError:
    try:
        from quanser.communications import Timeout
    except ImportError:
        print("[StreamClient] ERROR: quanser module not installed")
        sys.exit(1)

from uparking_camera import UParkingCamera

# ============================================================
# CONFIGURATION
# ============================================================

# Ground station IP — change to actual IP for hardware
GROUND_STATION_IP = "localhost"
STREAM_PORT = 18001
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CHANNELS = 3
BUFFER_SIZE = IMAGE_HEIGHT * IMAGE_WIDTH * IMAGE_CHANNELS
SAMPLE_RATE = 15.0  # fps for streaming (lower = more stable)
CAMERA_MODE = "csi"  # "csi", "realsense", or "qlabs"
CAMERA_ID = "4"  # 4 = downward camera on QDrone2

# ============================================================
# STREAM CLIENT
# ============================================================

class DroneStreamClient:
    """
    Captures frames from the drone camera and streams them
    to a ground station via TCP.

    Can also run a local CNN for edge classification and
    send just the results instead of raw frames.

    Usage:
        client = DroneStreamClient()
        client.connect()
        client.stream_loop(duration=60)
        client.stop()
    """

    def __init__(self, server_ip=GROUND_STATION_IP, port=STREAM_PORT,
                 camera_mode=CAMERA_MODE, camera_id=CAMERA_ID,
                 width=IMAGE_WIDTH, height=IMAGE_HEIGHT):

        self.server_ip = server_ip
        self.port = port
        self.width = width
        self.height = height
        self.buffer_size = height * width * IMAGE_CHANNELS

        # Camera
        self.camera = UParkingCamera(
            mode=camera_mode,
            width=width,
            height=height,
            camera_id=camera_id
        )

        # Stream
        self.stream = None
        self.connected = False
        self.timeout = Timeout(seconds=0, nanoseconds=1)

    def connect(self):
        """Initialize camera and connect to ground station."""
        # Start camera
        self.camera.start()

        # Create TCP client
        uri = f'tcpip://{self.server_ip}:{self.port}'
        self.stream = BasicStream(
            uri,
            agent='C',
            sendBufferSize=self.buffer_size,
            recvBufferSize=2048,
            nonBlocking=False
        )
        print(f"[StreamClient] Connecting to {uri}...")

        # Wait for server connection
        max_wait = 30  # seconds
        start = time.time()
        while not self.stream.connected and (time.time() - start) < max_wait:
            self.stream.checkConnection(timeout=self.timeout)
            time.sleep(0.1)

        if self.stream.connected:
            print("[StreamClient] Connected to ground station")
            self.connected = True
        else:
            print("[StreamClient] WARNING: Could not connect to ground station")
            self.connected = False

    def send_frame(self, frame):
        """Send a single frame to the ground station."""
        if not self.connected or self.stream is None:
            return False

        try:
            sent = self.stream.send(frame)
            return sent != -1
        except Exception as e:
            print(f"[StreamClient] Send error: {e}")
            return False

    def send_result(self, label, occupied, confidence):
        """Send just the classification result (lightweight)."""
        if not self.connected:
            return False

        result = json.dumps({
            "spot": label,
            "occupied": occupied,
            "confidence": float(confidence),
            "timestamp": time.time()
        }).encode('utf-8')

        try:
            sent = self.stream.send(np.frombuffer(result, dtype=np.uint8))
            return sent != -1
        except Exception:
            return False

    def stream_loop(self, duration=60.0, show_local=False):
        """
        Main streaming loop — capture and send frames.

        Parameters
        ----------
        duration   : seconds to stream
        show_local : if True, show camera feed locally too
        """
        sample_time = 1.0 / SAMPLE_RATE
        start = time.time()
        frame_count = 0
        fail_count = 0

        print(f"[StreamClient] Streaming at {SAMPLE_RATE} fps "
              f"for {duration}s...")

        try:
            while (time.time() - start) < duration:
                loop_start = time.time()

                # Capture frame
                frame = self.camera.capture()
                if frame is None:
                    fail_count += 1
                    if fail_count <= 5:
                        print(f"[StreamClient] Capture failed "
                              f"({fail_count})")
                    continue

                # Send to ground station
                if self.connected:
                    sent = self.send_frame(frame)
                    if not sent:
                        print("[StreamClient] Ground station not receiving")
                        break

                frame_count += 1

                # Optional local display
                if show_local:
                    cv2.imshow('Drone Downward Camera', frame)
                    cv2.waitKey(1)

                # Maintain frame rate
                elapsed = time.time() - loop_start
                sleep_time = sample_time - (elapsed % sample_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("[StreamClient] User interrupted")

        elapsed = time.time() - start
        print(f"\n[StreamClient] Streamed {frame_count} frames "
              f"in {elapsed:.1f}s ({frame_count/max(elapsed,1):.1f} fps)")
        print(f"  Failed captures: {fail_count}")

    def stop(self):
        """Clean up camera and stream."""
        self.camera.stop()
        if self.stream is not None:
            self.stream.terminate()
        print("[StreamClient] Stopped")


# ============================================================
# STANDALONE EXECUTION
# ============================================================

if __name__ == '__main__':
    client = DroneStreamClient()
    try:
        client.connect()
        client.stream_loop(duration=60, show_local=True)
    finally:
        client.stop()
