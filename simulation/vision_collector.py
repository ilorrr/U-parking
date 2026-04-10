# ============================================================
# vision_collector.py
# Captures drone downward-camera frames during scans and saves
# them with IoU-based labels for training a vision classifier.
#
# Usage:
#   collector = VisionCollector(drone, save_dir="vision_data")
#   collector.capture(spot_label="A1", occupied=True)
#   ...
#   collector.summary()
# ============================================================

import os
import cv2
import json
import time
import numpy as np
from datetime import datetime


# Camera constant (must match qdrone2.py)
CAMERA_DOWNWARD = 5

# Default crop size for spot patches (pixels)
PATCH_SIZE = 128

# Minimum frames between captures at same spot (avoid duplicates)
MIN_INTERVAL = 0.5


class VisionCollector:
    """
    Hooks into the drone scan loop to collect labeled training images.

    For each waypoint the drone visits, captures the downward camera
    frame, crops the center patch (the spot below), and saves it
    into occupied/ or empty/ subdirectories with metadata.
    """

    def __init__(self, drone, save_dir="vision_data", patch_size=PATCH_SIZE,
                 camera=CAMERA_DOWNWARD, sub_dir=None):
        self.drone      = drone
        self.save_dir   = save_dir
        self.patch_size = patch_size
        self.camera     = camera
        self.count      = {"occupied": 0, "empty": 0}
        self._last_time = 0

        # Support per-drone subdirectories for multi-drone collection
        base = os.path.join(save_dir, sub_dir) if sub_dir else save_dir

        # Create directory structure
        self._occ_dir   = os.path.join(base, "occupied")
        self._empty_dir = os.path.join(base, "empty")
        self._meta_file = os.path.join(base, "metadata.json")
        os.makedirs(self._occ_dir, exist_ok=True)
        os.makedirs(self._empty_dir, exist_ok=True)

        # Load or initialize metadata
        if os.path.exists(self._meta_file):
            with open(self._meta_file, "r") as f:
                self._metadata = json.load(f)
        else:
            self._metadata = {"captures": [], "created": datetime.now().isoformat()}

        existing_occ   = len(os.listdir(self._occ_dir))
        existing_empty = len(os.listdir(self._empty_dir))
        self.count["occupied"] = existing_occ
        self.count["empty"]    = existing_empty

        print(f"  [Vision] Collector ready -> {save_dir}")
        if existing_occ + existing_empty > 0:
            print(f"  [Vision] Existing data: {existing_occ} occupied, "
                  f"{existing_empty} empty")

    def _crop_center(self, frame):
        """Crop the center patch from the downward camera frame."""
        h, w = frame.shape[:2]
        ps = self.patch_size
        cx, cy = w // 2, h // 2
        x1 = max(0, cx - ps // 2)
        y1 = max(0, cy - ps // 2)
        x2 = min(w, x1 + ps)
        y2 = min(h, y1 + ps)
        patch = frame[y1:y2, x1:x2]

        # Resize to exact patch_size if crop was clipped at edges
        if patch.shape[0] != ps or patch.shape[1] != ps:
            patch = cv2.resize(patch, (ps, ps), interpolation=cv2.INTER_LINEAR)
        return patch

    def capture(self, spot_label, occupied, spot_xy=None, drone_id=None):
        """
        Capture a frame from the drone's downward camera, crop the
        center patch, and save it with the given label.

        Args:
            spot_label: parking spot ID (e.g. "A1")
            occupied:   True/False from IoU-based detection
            spot_xy:    optional (x, y) of spot center for metadata
            drone_id:   optional drone actor number for multi-drone setups
        Returns:
            True if capture was saved, False if skipped/failed
        """
        now = time.time()
        if now - self._last_time < MIN_INTERVAL:
            return False
        self._last_time = now

        # Capture frame from drone camera
        try:
            success, cam_num, frame = self.drone.get_image(self.camera)
            if not success or frame is None:
                return False
        except Exception as e:
            print(f"  [Vision] Capture failed at {spot_label}: {e}")
            return False

        # Crop center patch
        patch = self._crop_center(frame)

        # Determine save path
        class_name = "occupied" if occupied else "empty"
        class_dir  = self._occ_dir if occupied else self._empty_dir
        idx        = self.count[class_name]
        filename   = f"{spot_label}_{idx:05d}.jpg"
        filepath   = os.path.join(class_dir, filename)

        # Save patch
        cv2.imwrite(filepath, patch, [cv2.IMWRITE_JPEG_QUALITY, 95])
        self.count[class_name] += 1

        # Update metadata
        entry = {
            "file"     : os.path.join(class_name, filename),
            "label"    : spot_label,
            "occupied" : occupied,
            "timestamp": datetime.now().isoformat(),
        }
        if spot_xy is not None:
            entry["x"], entry["y"] = float(spot_xy[0]), float(spot_xy[1])
        if drone_id is not None:
            entry["drone"] = drone_id
        self._metadata["captures"].append(entry)

        return True

    def capture_full_frame(self, spot_label, occupied, spot_xy=None):
        """
        Save the FULL downward camera frame (not cropped).
        Useful for building a YOLO-style dataset where you annotate
        bounding boxes across the entire field of view.

        Saved to vision_data/full_frames/ with metadata.
        """
        full_dir = os.path.join(self.save_dir, "full_frames")
        os.makedirs(full_dir, exist_ok=True)

        try:
            success, cam_num, frame = self.drone.get_image(self.camera)
            if not success or frame is None:
                return False
        except Exception as e:
            print(f"  [Vision] Full-frame capture failed at {spot_label}: {e}")
            return False

        total = self.count["occupied"] + self.count["empty"]
        filename = f"frame_{total:05d}_{spot_label}.jpg"
        filepath = os.path.join(full_dir, filename)
        cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

        return True

    def save_metadata(self):
        """Flush metadata to disk."""
        self._metadata["total_occupied"] = self.count["occupied"]
        self._metadata["total_empty"]    = self.count["empty"]
        self._metadata["last_updated"]   = datetime.now().isoformat()
        with open(self._meta_file, "w") as f:
            json.dump(self._metadata, f, indent=2)

    def summary(self):
        """Print collection stats and save metadata."""
        self.save_metadata()
        total = self.count["occupied"] + self.count["empty"]
        print(f"\n  [Vision] Collection complete:")
        print(f"    Total:    {total}")
        print(f"    Occupied: {self.count['occupied']}")
        print(f"    Empty:    {self.count['empty']}")
        if total > 0:
            ratio = self.count["occupied"] / total * 100
            print(f"    Ratio:    {ratio:.1f}% occupied")
        print(f"    Saved to: {self.save_dir}")
        return self.count
