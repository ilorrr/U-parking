"""
occupancy.py
------------
IoU-based occupancy matching and lot-level statistics for uParking.

Adapted from:
    Peraza-Garzón et al. (2026) - "Intelligent Car Park Occupancy Monitoring
    System Based on Parking Slot and Vehicle Detection Using DJI Mini 3
    Aerial Imagery and YOLOv11"
    DOI: https://doi.org/10.3390/ai7020074

Key design decisions from Paper 2:
    - IoU threshold of 0.10–0.20 (we use 0.15) rather than the standard
      0.50, because aerial perspective distortion causes bounding boxes
      of vehicles and parking slots to misalign at altitude.
    - Occupancy % formula:  N_occupied / (N_occupied + N_free) × 100
      (Equation 7 in the paper).
    - A slot is OCCUPIED if its IoU with ANY detected vehicle exceeds the
      threshold — one match is sufficient.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np

from .detector import Detection, DetectionResult


# -- Constants -----------------------------------------------------------------

# Aerial IoU threshold — lower than the standard 0.50 used in ground-level
# detection, because altitude and gimbal angle introduce bounding-box
# misalignment between vehicle and slot detections.
# Source: Peraza-Garzón et al. (2026), Section 3.2.8, range 0.10–0.20.
AERIAL_IOU_THRESHOLD: float = 0.15

# Total number of spaces in the uParking simulation lot
TOTAL_LOT_SPACES: int = 156


# -- Data classes --------------------------------------------------------------

@dataclass
class SlotState:
    """
    Occupancy state of a single parking slot.

    Attributes
    ----------
    slot_id       : Unique integer identifier for the slot within the frame
                    (assigned by detection order from YOLOv11).
    bbox          : [x1, y1, x2, y2] in 640×640 preprocessed frame space.
    occupied      : True if a vehicle was matched to this slot via IoU.
    matched_iou   : Highest IoU found between this slot and any vehicle.
                    0.0 if no vehicle overlapped.
    slot_conf     : YOLOv11 confidence score for the parking-slot detection.
    waypoint_id   : Which drone waypoint this detection originated from.
    """
    slot_id: int
    bbox: List[float]
    occupied: bool
    matched_iou: float = 0.0
    slot_conf: float = 0.0
    waypoint_id: Optional[int] = None

    @property
    def state_label(self) -> str:
        return "occupied" if self.occupied else "available"


@dataclass
class LotSummary:
    """
    Lot-wide occupancy statistics broadcast to the React dashboard.

    Attributes
    ----------
    total_spaces      : Total spaces in the lot (156 for uParking).
    occupied          : Number of occupied spaces detected so far.
    available         : Number of available spaces detected so far.
    unscanned         : Spaces not yet seen by the drone in this patrol.
    occupancy_pct     : Equation 7 from Paper 2 (occupied / total seen × 100).
    slot_states       : Full per-slot occupancy map for dashboard rendering.
    """
    total_spaces: int = TOTAL_LOT_SPACES
    occupied: int = 0
    available: int = 0
    unscanned: int = field(init=False)
    occupancy_pct: float = 0.0
    slot_states: Dict[int, SlotState] = field(default_factory=dict)

    def __post_init__(self):
        self.unscanned = self.total_spaces - self.occupied - self.available

    def to_dict(self) -> dict:
        """Serialise for WebSocket JSON broadcast."""
        return {
            "total_spaces": self.total_spaces,
            "occupied": self.occupied,
            "available": self.available,
            "unscanned": self.unscanned,
            "occupancy_pct": round(self.occupancy_pct, 2),
            "slot_states": {
                sid: {
                    "bbox": s.bbox,
                    "occupied": s.occupied,
                    "state": s.state_label,
                    "iou": round(s.matched_iou, 4),
                    "confidence": round(s.slot_conf, 4),
                    "waypoint": s.waypoint_id
                }
                for sid, s in self.slot_states.items()
            }
        }


# -- Core functions -------------------------------------------------------------

def compute_iou(box_a: List[float], box_b: List[float]) -> float:
    """
    Compute Intersection over Union between two bounding boxes.

    Both boxes must be in [x1, y1, x2, y2] format.

    Implements Equation 4 from Peraza-Garzón et al. (2026):

        IoU = |B_pred ∩ B_gt| / |B_pred ∪ B_gt|

    Parameters
    ----------
    box_a, box_b : Bounding boxes as [x1, y1, x2, y2].

    Returns
    -------
    float in [0, 1].  Returns 0 for degenerate (zero-area) boxes.
    """
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])

    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)

    area_a = max(0.0, box_a[2] - box_a[0]) * max(0.0, box_a[3] - box_a[1])
    area_b = max(0.0, box_b[2] - box_b[0]) * max(0.0, box_b[3] - box_b[1])
    union  = area_a + area_b - intersection

    if union <= 0.0:
        return 0.0

    return intersection / union


def match_occupancy(
    detection_result: DetectionResult,
    iou_threshold: float = AERIAL_IOU_THRESHOLD,
    waypoint_id: Optional[int] = None
) -> Dict[int, SlotState]:
    """
    Match vehicle detections to parking slots using IoU.

    A slot is marked OCCUPIED when its IoU with at least one detected
    vehicle exceeds iou_threshold.  The lower default threshold (0.15)
    is used specifically for aerial imagery where perspective distortion
    reduces bounding-box overlap precision.

    Source: Peraza-Garzón et al. (2026), Section 3.2.8, steps 1–3.

    Parameters
    ----------
    detection_result : Output from ParkingDetector.detect().
    iou_threshold    : Minimum IoU to classify a slot as occupied.
                       Default 0.15 (midpoint of Paper 2's 0.10–0.20 range).
    waypoint_id      : Drone waypoint index, recorded for debugging.

    Returns
    -------
    Dict mapping slot_id → SlotState for every detected parking slot.

    Example
    -------
    >>> result = detector.detect(frame)
    >>> states = match_occupancy(result, waypoint_id=3)
    >>> for sid, state in states.items():
    ...     print(sid, state.state_label)
    """
    slot_states: Dict[int, SlotState] = {}

    for slot_id, slot in enumerate(detection_result.parking_slots):
        best_iou = 0.0

        # Check each detected vehicle against this slot
        for vehicle in detection_result.vehicles:
            iou = compute_iou(slot.bbox, vehicle.bbox)
            if iou > best_iou:
                best_iou = iou

        is_occupied = best_iou > iou_threshold

        slot_states[slot_id] = SlotState(
            slot_id=slot_id,
            bbox=slot.bbox,
            occupied=is_occupied,
            matched_iou=best_iou,
            slot_conf=slot.confidence,
            waypoint_id=waypoint_id
        )

    return slot_states


def compute_occupancy_percentage(
    slot_states: Dict[int, SlotState]
) -> float:
    """
    Compute overall parking lot occupancy as a percentage.

    Implements Equation 7 from Peraza-Garzón et al. (2026):

        Occupancy(%) = N_occupied / (N_occupied + N_free) × 100

    Only counts slots that have been actively detected by the drone
    (i.e. entries in slot_states).  Unscanned slots are not assumed
    empty or occupied.

    Parameters
    ----------
    slot_states : Dict from match_occupancy().

    Returns
    -------
    float in [0, 100].  Returns 0 if no slots have been scanned yet.
    """
    if not slot_states:
        return 0.0

    n_occupied = sum(1 for s in slot_states.values() if s.occupied)
    n_free     = len(slot_states) - n_occupied

    total = n_occupied + n_free
    if total == 0:
        return 0.0

    return (n_occupied / total) * 100.0


def build_lot_summary(
    slot_states: Dict[int, SlotState],
    total_spaces: int = TOTAL_LOT_SPACES
) -> LotSummary:
    """
    Build a LotSummary for WebSocket broadcast from the current slot states.

    Parameters
    ----------
    slot_states   : Merged per-slot states from all scanned waypoints.
    total_spaces  : Total spaces in the lot.  Default 156 for uParking.

    Returns
    -------
    LotSummary ready for .to_dict() serialisation.
    """
    n_occupied = sum(1 for s in slot_states.values() if s.occupied)
    n_available = len(slot_states) - n_occupied
    pct = compute_occupancy_percentage(slot_states)

    summary = LotSummary(
        total_spaces=total_spaces,
        occupied=n_occupied,
        available=n_available,
        occupancy_pct=pct,
        slot_states=slot_states
    )
    # Recalculate unscanned after init
    summary.unscanned = total_spaces - len(slot_states)
    return summary


# -- Utility -------------------------------------------------------------------

def merge_waypoint_states(
    existing: Dict[int, SlotState],
    new_states: Dict[int, SlotState],
    id_offset: int = 0
) -> Dict[int, SlotState]:
    """
    Merge slot states from a new waypoint into the persistent lot map.

    Since different drone waypoints cover different zones of the 156-space
    lot, slot IDs from each waypoint need an offset to avoid collision in
    the global map.

    Parameters
    ----------
    existing   : Running lot-wide slot state dict (updated in place).
    new_states : States returned by match_occupancy() at current waypoint.
    id_offset  : Integer offset so this waypoint's slots don't overwrite
                 slot IDs from previous waypoints.

    Returns
    -------
    Updated existing dict (also mutated in place for efficiency).
    """
    for local_id, state in new_states.items():
        global_id = local_id + id_offset
        # If slot already seen, update only if new detection has higher
        # confidence (more recent / better angle)
        if global_id in existing:
            if state.slot_conf > existing[global_id].slot_conf:
                state.slot_id = global_id
                existing[global_id] = state
        else:
            state.slot_id = global_id
            existing[global_id] = state

    return existing
