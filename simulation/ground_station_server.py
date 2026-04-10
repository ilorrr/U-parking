# ============================================================
# ground_station_server.py
# Runs on the ground station PC.
# Receives frames from the QDrone2 via Quanser BasicStream TCP,
# classifies each frame with the trained CNN,
# and pushes results to the Django backend.
#
# Also displays the live drone camera feed for monitoring.
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
        print("[GroundStation] ERROR: quanser module not installed")
        sys.exit(1)

# ============================================================
# CONFIGURATION
# ============================================================

STREAM_PORT = 18001
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CHANNELS = 3
BUFFER_SIZE = IMAGE_HEIGHT * IMAGE_WIDTH * IMAGE_CHANNELS
MODEL_PATH = "vision_model.pth"
BACKEND_URL = "http://localhost:8000/api/occupancy-update/"

# ============================================================
# CNN CLASSIFIER (optional — works without trained model too)
# ============================================================

class ParkingClassifier:
    """
    Loads the trained CNN and classifies parking spot images.
    Falls back to a simple brightness heuristic if no model.
    """

    def __init__(self, model_path=MODEL_PATH):
        self.model = None
        self.device = "cpu"

        try:
            import torch
            from torchvision import transforms

            self.torch = torch
            self.transform = transforms.Compose([
                transforms.ToPILImage(),
                transforms.Resize((128, 128)),
                transforms.ToTensor(),
            ])

            checkpoint = torch.load(model_path, map_location="cpu")
            if "model_state_dict" in checkpoint:
                # Load model architecture
                from vision_model import ParkingSpotCNN
                self.model = ParkingSpotCNN()
                self.model.load_state_dict(checkpoint["model_state_dict"])
            else:
                self.model = checkpoint

            self.model.eval()
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            self.model.to(self.device)
            print(f"[Classifier] CNN loaded from {model_path} "
                  f"(device: {self.device})")

        except FileNotFoundError:
            print(f"[Classifier] No model at {model_path} — "
                  f"using brightness heuristic")
        except Exception as e:
            print(f"[Classifier] Could not load model: {e} — "
                  f"using brightness heuristic")

    def classify(self, frame):
        """
        Classify a frame as occupied (True) or empty (False).
        Returns (occupied, confidence).
        """
        if frame is None:
            return False, 0.0

        # Center crop 128x128
        h, w = frame.shape[:2]
        cy, cx = h // 2, w // 2
        patch = frame[cy-64:cy+64, cx-64:cx+64]

        if self.model is not None:
            return self._cnn_classify(patch)
        else:
            return self._heuristic_classify(patch)

    def _cnn_classify(self, patch):
        """CNN-based classification."""
        rgb = cv2.cvtColor(patch, cv2.COLOR_BGR2RGB)
        tensor = self.transform(rgb).unsqueeze(0).to(self.device)

        with self.torch.no_grad():
            output = self.model(tensor)
            prob = self.torch.sigmoid(output).item()

        occupied = prob > 0.5
        confidence = prob if occupied else (1.0 - prob)
        return occupied, confidence

    def _heuristic_classify(self, patch):
        """
        Simple brightness heuristic — colored vehicles are darker
        than gray pavement when viewed from above.
        """
        gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)

        # Pavement is typically bright (>150), vehicles darker (<130)
        occupied = mean_brightness < 130
        confidence = abs(mean_brightness - 130) / 130
        return occupied, min(confidence, 1.0)


# ============================================================
# BACKEND PUSH
# ============================================================

def push_to_backend(results, url=BACKEND_URL):
    """Push accumulated results to Django backend."""
    try:
        import requests
    except ImportError:
        print("[Backend] requests not installed — skipping push")
        return

    for section in [1, 2]:
        spots = {
            label: occ
            for label, occ in results.items()
            if label.startswith(f"S{section}-")
        }
        if not spots:
            continue

        try:
            response = requests.post(url, json={
                "section": section,
                "spots": spots
            }, timeout=5)
            occ = sum(1 for v in spots.values() if v)
            print(f"  [Backend] S{section}: {occ}/{len(spots)} "
                  f"occupied → {response.status_code}")
        except Exception as e:
            print(f"  [Backend] S{section} push failed: {e}")


# ============================================================
# GROUND STATION SERVER
# ============================================================

class GroundStationServer:
    """
    Receives drone camera frames via TCP, classifies them,
    displays the live feed, and pushes results to the backend.

    Usage:
        server = GroundStationServer()
        server.start()
        server.receive_loop(duration=120)
        server.stop()
    """

    def __init__(self, port=STREAM_PORT, model_path=MODEL_PATH):
        self.port = port
        self.image_data = np.zeros(
            (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS),
            dtype=np.uint8
        )
        self.stream = None
        self.connected = False
        self.classifier = ParkingClassifier(model_path)
        self.results = {}  # label -> occupied
        self.timeout = Timeout(seconds=0, nanoseconds=1)

    def start(self):
        """Start the TCP server and wait for drone connection."""
        uri = f'tcpip://localhost:{self.port}'
        self.stream = BasicStream(
            uri,
            agent='S',
            sendBufferSize=BUFFER_SIZE,
            recvBufferSize=BUFFER_SIZE,
            receiveBuffer=self.image_data,
            nonBlocking=False
        )
        print(f"[GroundStation] Server listening on {uri}")
        print("[GroundStation] Waiting for drone to connect...")

    def receive_loop(self, duration=120.0, spot_labels=None):
        """
        Main receive loop — get frames, classify, display.

        Parameters
        ----------
        duration     : seconds to run
        spot_labels  : optional list of spot labels to cycle through
        """
        start = time.time()
        frame_count = 0
        spot_index = 0

        try:
            while (time.time() - start) < duration:
                # Check for drone connection
                if not self.stream.connected:
                    self.stream.checkConnection(timeout=self.timeout)
                    if self.stream.connected:
                        print("[GroundStation] Drone connected!")
                        self.connected = True
                    continue

                # Receive frame
                recv_flag, bytes_received = self.stream.receive(
                    iterations=2, timeout=self.timeout)

                if not recv_flag:
                    continue

                frame = self.stream.receiveBuffer.copy()
                frame_count += 1

                # Classify
                occupied, confidence = self.classifier.classify(frame)

                # Assign spot label if available
                if spot_labels and spot_index < len(spot_labels):
                    label = spot_labels[spot_index]
                    self.results[label] = occupied
                    spot_index += 1

                    status = "OCCUPIED" if occupied else "EMPTY"
                    print(f"  [{label}] {status} "
                          f"(conf: {confidence:.2f})")

                # Draw overlay on frame
                color = (0, 0, 255) if occupied else (0, 255, 0)
                text = f"{'OCCUPIED' if occupied else 'EMPTY'} "
                text += f"({confidence:.0%})"
                cv2.putText(frame, text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            color, 2)

                if spot_labels and spot_index <= len(spot_labels):
                    idx = min(spot_index - 1, len(spot_labels) - 1)
                    cv2.putText(frame, spot_labels[idx], (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 255), 1)

                cv2.imshow('Drone Live Feed', frame)
                cv2.waitKey(1)

                # Push to backend every 50 frames
                if frame_count % 50 == 0 and self.results:
                    push_to_backend(self.results)

        except KeyboardInterrupt:
            print("\n[GroundStation] User interrupted")

        # Final push
        if self.results:
            print("\n[GroundStation] Final backend push...")
            push_to_backend(self.results)

        elapsed = time.time() - start
        occ = sum(1 for v in self.results.values() if v)
        print(f"\n[GroundStation] Received {frame_count} frames "
              f"in {elapsed:.1f}s")
        print(f"  Classified: {len(self.results)} spots | "
              f"Occupied: {occ} | Empty: {len(self.results) - occ}")

    def stop(self):
        """Clean up."""
        if self.stream is not None:
            self.stream.terminate()
        cv2.destroyAllWindows()
        print("[GroundStation] Stopped")


# ============================================================
# STANDALONE EXECUTION
# ============================================================

if __name__ == '__main__':
    server = GroundStationServer()
    try:
        server.start()
        server.receive_loop(duration=120)
    finally:
        server.stop()
