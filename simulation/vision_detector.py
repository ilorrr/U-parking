# ============================================================
# vision_detector.py
# Drop-in vision-based occupancy detector for the drone scanner.
#
# Replaces the coordinate/IoU-based _spot_occupied() with a
# CNN that classifies the drone's downward camera feed.
#
# Usage in drone_scanner.py:
#   from vision_detector import VisionDetector
#   detector = VisionDetector(drone, model_path="vision_model.pth")
#   occupied, confidence = detector.predict()
# ============================================================

import os
import cv2
import numpy as np

# Camera constant (must match qdrone2.py)
CAMERA_DOWNWARD = 5

# Default confidence threshold — above this = occupied
CONFIDENCE_THRESHOLD = 0.5

# Patch size (must match training)
PATCH_SIZE = 128

# ---------- Check for PyTorch ----------
try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False


class VisionDetector:
    """
    Vision-based parking spot occupancy detector.

    Captures the downward camera frame, crops the center patch,
    and runs it through the trained CNN to predict occupied/empty.

    Can be used as a drop-in replacement for _spot_occupied() or
    combined with IoU detection for higher confidence.
    """

    def __init__(self, drone, model_path="vision_model.pth",
                 threshold=CONFIDENCE_THRESHOLD, camera=CAMERA_DOWNWARD):
        self.drone     = drone
        self.threshold = threshold
        self.camera    = camera
        self.model     = None
        self.device    = None
        self._ready    = False

        if not HAS_TORCH:
            print("  [VisionDet] WARNING: PyTorch not installed. "
                  "Vision detection disabled.")
            return

        if not os.path.exists(model_path):
            print(f"  [VisionDet] WARNING: Model not found at {model_path}. "
                  f"Run vision_model.py to train first.")
            return

        # Load model
        from vision_model import ParkingSpotCNN
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        self.model = ParkingSpotCNN().to(self.device)

        checkpoint = torch.load(model_path, map_location=self.device,
                                weights_only=True)
        self.model.load_state_dict(checkpoint["model_state_dict"])
        self.model.eval()
        self._ready = True

        acc = checkpoint.get("val_acc", 0)
        print(f"  [VisionDet] Model loaded: {model_path} "
              f"(val acc: {acc:.1%}, threshold: {threshold})")

    @property
    def ready(self):
        return self._ready

    def _capture_patch(self):
        """Capture and crop center patch from downward camera."""
        try:
            success, _, frame = self.drone.get_image(self.camera)
            if not success or frame is None:
                return None
        except Exception:
            return None

        h, w = frame.shape[:2]
        ps = PATCH_SIZE
        cx, cy = w // 2, h // 2
        x1 = max(0, cx - ps // 2)
        y1 = max(0, cy - ps // 2)
        x2 = min(w, x1 + ps)
        y2 = min(h, y1 + ps)
        patch = frame[y1:y2, x1:x2]

        if patch.shape[0] != ps or patch.shape[1] != ps:
            patch = cv2.resize(patch, (ps, ps))
        return patch

    def predict(self):
        """
        Capture a frame and predict occupancy.

        Returns:
            (occupied: bool, confidence: float)
            Returns (False, 0.0) if detection is unavailable.
        """
        if not self._ready:
            return False, 0.0

        patch = self._capture_patch()
        if patch is None:
            return False, 0.0

        # Preprocess: BGR → RGB, normalize, add batch dim
        rgb = cv2.cvtColor(patch, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
        tensor = tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            confidence = self.model(tensor).item()

        occupied = confidence >= self.threshold
        return occupied, confidence

    def predict_from_frame(self, frame):
        """
        Predict from an already-captured frame (skip camera capture).
        Useful for batch processing or testing.
        """
        if not self._ready:
            return False, 0.0

        h, w = frame.shape[:2]
        ps = PATCH_SIZE
        cx, cy = w // 2, h // 2
        x1 = max(0, cx - ps // 2)
        y1 = max(0, cy - ps // 2)
        patch = frame[y1:y1+ps, x1:x1+ps]
        if patch.shape[0] != ps or patch.shape[1] != ps:
            patch = cv2.resize(patch, (ps, ps))

        rgb = cv2.cvtColor(patch, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
        tensor = tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            confidence = self.model(tensor).item()

        return confidence >= self.threshold, confidence


# ============================================================
# HYBRID DETECTOR  -  combines vision + IoU for robustness
# ============================================================

class HybridDetector:
    """
    Combines the CNN vision detector with the existing IoU-based
    detection for higher reliability.

    Strategies:
      "vision_primary"  - use vision, fall back to IoU if model unavailable
      "iou_primary"     - use IoU, use vision as confirmation
      "both_agree"      - only mark occupied if BOTH methods agree
      "either"          - mark occupied if EITHER method detects it
    """

    def __init__(self, drone, model_path="vision_model.pth",
                 strategy="vision_primary", threshold=CONFIDENCE_THRESHOLD):
        self.vision   = VisionDetector(drone, model_path, threshold)
        self.strategy = strategy
        print(f"  [HybridDet] Strategy: {strategy}")

    def predict(self, spot, static_positions, live_positions):
        """
        Combined prediction using vision and IoU.

        Args:
            spot:             parking spot dict from parking_lot.py
            static_positions: [[x, y, name], ...] from vehicle_actors
            live_positions:   [[x, y], ...] from QCar poller

        Returns:
            (occupied: bool, method: str, confidence: float)
        """
        from drone_scanner import _spot_occupied

        # Get both predictions
        vision_occ, vision_conf = self.vision.predict() if self.vision.ready else (False, 0.0)
        iou_occ = _spot_occupied(spot, static_positions, live_positions)

        if self.strategy == "vision_primary":
            if self.vision.ready:
                return vision_occ, "vision", vision_conf
            return iou_occ, "iou", 1.0 if iou_occ else 0.0

        elif self.strategy == "iou_primary":
            return iou_occ, "iou", 1.0 if iou_occ else 0.0

        elif self.strategy == "both_agree":
            agreed = vision_occ and iou_occ
            method = "both" if vision_occ == iou_occ else "disagree"
            return agreed, method, vision_conf

        elif self.strategy == "either":
            either = vision_occ or iou_occ
            if vision_occ and iou_occ:
                method = "both"
            elif vision_occ:
                method = "vision"
            elif iou_occ:
                method = "iou"
            else:
                method = "none"
            return either, method, vision_conf

        # Default fallback
        return iou_occ, "iou", 1.0 if iou_occ else 0.0
