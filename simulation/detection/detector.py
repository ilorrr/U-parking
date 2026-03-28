"""
detector.py
-----------
YOLOv11 dual-class parking detector for uParking.

Adapted from:
    Peraza-Garzón et al. (2026) - "Intelligent Car Park Occupancy Monitoring
    System Based on Parking Slot and Vehicle Detection Using DJI Mini 3
    Aerial Imagery and YOLOv11"
    DOI: https://doi.org/10.3390/ai7020074

Two detection classes (Paper 2, Section 3.2.2):
    0 → vehicle  — an occupied parking space (car present)
    1 → parking  — an unoccupied / free slot  (empty marked space)

Explicitly training the model on both classes is the key insight from
Paper 2.  Earlier approaches only detected vehicles and inferred vacancy
from the absence of a detection, which is unreliable.  By learning what
an empty aerial parking slot looks like the model is much more robust.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np

# Ultralytics YOLOv11 — install with: pip install ultralytics
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False
    print(
        "[detector] WARNING: ultralytics not installed. "
        "Run `pip install ultralytics` to enable real inference. "
        "MockDetector will be used instead."
    )


# -- Constants -----------------------------------------------------------------

CLASS_VEHICLE: int = 0   # occupied space
CLASS_PARKING: int = 1   # free / empty slot

# Confidence threshold recommended for deployment in Paper 2.
# Predictions below this are discarded before IoU matching.
DEFAULT_CONFIDENCE: float = 0.50


# -- Data classes --------------------------------------------------------------

@dataclass
class Detection:
    """
    Single bounding-box detection from YOLOv11.

    Attributes
    ----------
    bbox        : [x1, y1, x2, y2] in the 640×640 preprocessed frame space.
    confidence  : Model confidence score in [0, 1].
    class_id    : 0 = vehicle, 1 = parking (free slot).
    class_name  : Human-readable label.
    """
    bbox: List[float]
    confidence: float
    class_id: int
    class_name: str = field(init=False)

    # Class id → label mapping  (Paper 2 dual-class formulation)
    _LABELS = {CLASS_VEHICLE: "vehicle", CLASS_PARKING: "parking"}

    def __post_init__(self):
        self.class_name = self._LABELS.get(self.class_id, "unknown")

    @property
    def area(self) -> float:
        """Bounding-box area in pixels² (640×640 space)."""
        x1, y1, x2, y2 = self.bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    @property
    def center(self) -> Tuple[float, float]:
        """Centre (cx, cy) of the bounding box."""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2, (y1 + y2) / 2)


@dataclass
class DetectionResult:
    """
    All detections from a single frame, split by class.

    Attributes
    ----------
    vehicles        : List of vehicle detections  (class 0).
    parking_slots   : List of parking-slot detections (class 1).
    raw_frame_shape : (H, W) of the 640×640 input tensor.
    """
    vehicles: List[Detection]
    parking_slots: List[Detection]
    raw_frame_shape: Tuple[int, int] = (640, 640)

    @property
    def n_vehicles(self) -> int:
        return len(self.vehicles)

    @property
    def n_slots(self) -> int:
        return len(self.parking_slots)


# -- Detector ------------------------------------------------------------------

class ParkingDetector:
    """
    YOLOv11 dual-class parking space detector.

    Usage
    -----
    >>> detector = ParkingDetector("path/to/best.pt")
    >>> result = detector.detect(preprocessed_frame)
    >>> print(result.n_vehicles, result.n_slots)

    Parameters
    ----------
    model_path   : Path to a .pt weights file fine-tuned on the dual-class
                   parking dataset (vehicle + parking classes).
                   Use 'yolo11n.pt' for the base nano model during development.
    confidence   : Confidence threshold.  Detections below this are ignored.
                   Paper 2 uses 0.50 for deployment.
    """

    def __init__(
        self,
        model_path: str = "yolo11n.pt",
        confidence: float = DEFAULT_CONFIDENCE
    ):
        if not ULTRALYTICS_AVAILABLE:
            raise RuntimeError(
                "ultralytics package not found. "
                "Install with: pip install ultralytics"
            )

        self.confidence = confidence
        self.model = YOLO(model_path)
        print(f"[ParkingDetector] Loaded model: {model_path}")
        print(f"[ParkingDetector] Confidence threshold: {confidence}")

    def detect(self, frame: np.ndarray) -> DetectionResult:
        """
        Run YOLOv11 inference on a preprocessed 640×640 frame.

        Parameters
        ----------
        frame : Float32 numpy array (640, 640, 3), values in [0, 1].
                Use preprocessing.preprocess_frame() to prepare frames.

        Returns
        -------
        DetectionResult with vehicles and parking_slots separated.

        Notes
        -----
        Paper 2 trains on two classes only:
            class 0 → vehicle  (occupied)
            class 1 → parking  (empty slot)
        Any other class ids are ignored.
        """
        results = self.model(
            frame,
            conf=self.confidence,
            verbose=False
        )[0]

        vehicles: List[Detection] = []
        parking_slots: List[Detection] = []

        for box in results.boxes:
            cls_id   = int(box.cls[0])
            conf     = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            det = Detection(
                bbox=[x1, y1, x2, y2],
                confidence=conf,
                class_id=cls_id
            )

            if cls_id == CLASS_VEHICLE:
                vehicles.append(det)
            elif cls_id == CLASS_PARKING:
                parking_slots.append(det)
            # silently skip unknown classes

        return DetectionResult(
            vehicles=vehicles,
            parking_slots=parking_slots
        )


# -- Mock detector for simulation / unit testing -------------------------------

class MockParkingDetector:
    """
    Drop-in replacement for ParkingDetector that returns synthetic detections.

    Use this during QLabs simulation development when you do not yet have
    fine-tuned YOLOv11 weights or when running unit tests without a GPU.

    The mock randomly populates a grid of parking slots and vehicles so the
    downstream occupancy logic can be tested end-to-end.

    Parameters
    ----------
    n_slots        : Number of parking slots to simulate per frame.
    occupancy_rate : Fraction of slots that will have a vehicle (0–1).
    seed           : Random seed for reproducibility.
    """

    def __init__(
        self,
        n_slots: int = 12,
        occupancy_rate: float = 0.5,
        seed: Optional[int] = 42
    ):
        self.n_slots = n_slots
        self.occupancy_rate = occupancy_rate
        self.rng = np.random.default_rng(seed)
        print(
            f"[MockParkingDetector] n_slots={n_slots}, "
            f"occupancy_rate={occupancy_rate}"
        )

    def detect(self, frame: np.ndarray) -> DetectionResult:
        """
        Generate a synthetic DetectionResult on a 640×640 grid.

        Slots are arranged in a regular grid.  A vehicle bounding box,
        slightly smaller than the slot, is placed inside each occupied slot.
        This ensures the IoU computation in occupancy.py works correctly.
        """
        cols = int(np.ceil(np.sqrt(self.n_slots)))
        rows = int(np.ceil(self.n_slots / cols))
        cell_w = 640 / cols
        cell_h = 640 / rows

        parking_slots: List[Detection] = []
        vehicles: List[Detection] = []

        slot_idx = 0
        for r in range(rows):
            for c in range(cols):
                if slot_idx >= self.n_slots:
                    break

                # Slot bounding box (full cell with small margin)
                margin = 4
                sx1 = c * cell_w + margin
                sy1 = r * cell_h + margin
                sx2 = (c + 1) * cell_w - margin
                sy2 = (r + 1) * cell_h - margin

                parking_slots.append(Detection(
                    bbox=[sx1, sy1, sx2, sy2],
                    confidence=float(self.rng.uniform(0.7, 0.99)),
                    class_id=CLASS_PARKING
                ))

                # Vehicle — placed inside slot with IoU > 0.15
                if self.rng.random() < self.occupancy_rate:
                    inner = 10
                    vehicles.append(Detection(
                        bbox=[sx1 + inner, sy1 + inner,
                              sx2 - inner, sy2 - inner],
                        confidence=float(self.rng.uniform(0.6, 0.95)),
                        class_id=CLASS_VEHICLE
                    ))

                slot_idx += 1

        return DetectionResult(
            vehicles=vehicles,
            parking_slots=parking_slots
        )
