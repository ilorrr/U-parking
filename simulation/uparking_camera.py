# ============================================================
# uparking_camera.py
# Camera interface using Quanser's Camera2D/Camera3D utilities.
# Bypasses qlabs get_image() — uses a separate communication
# path that may survive parallel scan pipe corruption.
#
# Works in BOTH simulation (QLabs) and hardware (physical QDrone2).
# ============================================================

import sys
import time
import cv2
import numpy as np


import sys
sys.path.insert(0, r"C:\Users\Roli\Documents\uparking")

# Quanser vision utilities
try:
    from pal.utilities.vision import Camera2D, Camera3D
    QUANSER_VISION_AVAILABLE = True
except ImportError:
    QUANSER_VISION_AVAILABLE = False
    print("[UParkingCamera] WARNING: pal.utilities.vision not available")


class UParkingCamera:
    """
    Unified camera interface for UParking.

    Supports three capture modes:
      - "csi"       : Camera2D — CSI downward camera (cameraId=4)
      - "realsense" : Camera3D — Intel RealSense RGB + Depth
      - "qlabs"     : drone.get_image() fallback (original method)

    Usage:
        cam = UParkingCamera(mode="csi")
        cam.start()

        frame = cam.capture()        # RGB numpy array
        depth = cam.capture_depth()  # depth in meters (realsense only)

        cam.stop()
    """

    def __init__(self, mode="csi", width=640, height=480, fps=30,
                 camera_id="4", qlabs_drone=None):
        """
        Parameters
        ----------
        mode       : "csi", "realsense", or "qlabs"
        width      : frame width
        height     : frame height
        fps        : frame rate
        camera_id  : CSI camera index ("4" = downward on QDrone2)
        qlabs_drone: QLabsQDrone2 object (only needed for "qlabs" mode)
        """
        self.mode = mode
        self.width = width
        self.height = height
        self.fps = fps
        self.camera_id = camera_id
        self.drone = qlabs_drone

        self._cam2d = None
        self._cam3d = None
        self._running = False

    def start(self):
        """Initialize the camera."""
        if self.mode == "csi":
            if not QUANSER_VISION_AVAILABLE:
                raise RuntimeError("Camera2D not available — install Quanser PAL")
            self._cam2d = Camera2D(
                cameraId=self.camera_id,
                frameWidth=self.width,
                frameHeight=self.height,
                frameRate=self.fps
            )
            print(f"[Camera] CSI camera {self.camera_id} started "
                  f"({self.width}x{self.height} @ {self.fps}fps)")

        elif self.mode == "realsense":
            if not QUANSER_VISION_AVAILABLE:
                raise RuntimeError("Camera3D not available — install Quanser PAL")
            self._cam3d = Camera3D(
                mode='RGB&DEPTH',
                frameWidthRGB=self.width,
                frameHeightRGB=self.height,
                frameRateRGB=self.fps,
                frameRateDepth=self.fps,
                frameWidthDepth=self.width,
                frameHeightDepth=self.height
            )
            print(f"[Camera] RealSense started "
                  f"({self.width}x{self.height} @ {self.fps}fps)")

        elif self.mode == "qlabs":
            if self.drone is None:
                raise RuntimeError("qlabs mode requires a drone object")
            print("[Camera] Using QLabs get_image() mode")

        else:
            raise ValueError(f"Unknown camera mode: {self.mode}")

        self._running = True

    def capture(self):
        """
        Capture a single RGB frame.
        Returns numpy array (H, W, 3) BGR format, or None on failure.
        """
        if not self._running:
            return None

        try:
            if self.mode == "csi":
                self._cam2d.read()
                return self._cam2d.imageData

            elif self.mode == "realsense":
                self._cam3d.read_RGB()
                return self._cam3d.imageBufferRGB

            elif self.mode == "qlabs":
                success, cam_num, frame = self.drone.get_image(5)
                if success and frame is not None:
                    return frame
                return None

        except Exception as e:
            print(f"[Camera] Capture error: {e}")
            return None

    def capture_depth(self):
        """
        Capture depth frame in meters (RealSense only).
        Returns numpy array (H, W) float32, or None.
        """
        if self.mode != "realsense" or self._cam3d is None:
            return None

        try:
            self._cam3d.read_depth(dataMode='M')
            return self._cam3d.imageBufferDepthM
        except Exception as e:
            print(f"[Camera] Depth capture error: {e}")
            return None

    def capture_with_depth(self):
        """
        Capture both RGB and depth simultaneously.
        Returns (rgb_frame, depth_frame) or (None, None).
        """
        rgb = self.capture()
        depth = self.capture_depth()
        return rgb, depth

    def stop(self):
        """Terminate the camera."""
        self._running = False
        try:
            if self._cam2d is not None:
                self._cam2d.terminate()
                print("[Camera] CSI camera terminated")
            if self._cam3d is not None:
                self._cam3d.terminate()
                print("[Camera] RealSense terminated")
        except Exception as e:
            print(f"[Camera] Terminate error: {e}")


class DepthDetector:
    """
    Depth-based parking spot detection using RealSense.

    A parked vehicle at 8m drone altitude reads ~6-7m depth.
    Empty pavement reads ~8m. No CNN needed.

    Usage:
        detector = DepthDetector(altitude=8.0)
        occupied, confidence = detector.classify(depth_frame)
    """

    def __init__(self, altitude=8.0, vehicle_height_min=0.5):
        self.altitude = altitude
        self.threshold = altitude - vehicle_height_min

    def classify(self, depth_frame, crop_center=True):
        """
        Classify a parking spot from depth data.

        Parameters
        ----------
        depth_frame  : numpy array (H, W) in meters
        crop_center  : if True, only use center 64x64 pixels

        Returns
        -------
        (occupied: bool, confidence: float)
        """
        if depth_frame is None:
            return False, 0.0

        h, w = depth_frame.shape
        if crop_center:
            cy, cx = h // 2, w // 2
            patch = depth_frame[cy-32:cy+32, cx-32:cx+32]
        else:
            patch = depth_frame

        # Filter out invalid readings (0 or NaN)
        valid = patch[(patch > 0.1) & (~np.isnan(patch))]
        if len(valid) == 0:
            return False, 0.0

        mean_depth = np.mean(valid)
        occupied = mean_depth < self.threshold

        # Confidence based on how far below threshold
        if occupied:
            confidence = min(1.0, (self.threshold - mean_depth) / 2.0)
        else:
            confidence = min(1.0, (mean_depth - self.threshold) / 2.0)

        return occupied, confidence
