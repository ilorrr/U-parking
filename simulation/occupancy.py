"""
occupancy.py
------------
Coordinate-based parking spot occupancy detection for uParking.

Used by:
    - main.py          (get_vehicle_positions, check_parking_spot_occupancy, etc.)
    - drone_scanner.py (bbox_iou, spot_vehicle_iou, is_spot_occupied, has_spillover, etc.)

NOTE: The YOLOv11 / Paper 2 vision-based occupancy logic lives in
      uparking_detection/occupancy.py — this file is the coordinate-based
      original and should remain at the project root.
"""

import math
from typing import List, Tuple, Optional

# ── Constants ─────────────────────────────────────────────────────────────────

# Primary IoU threshold for occupancy detection
IOU_THRESHOLD: float = 0.15

# Lower threshold for spillover detection (large vehicles straddling two spots)
SPILLOVER_IOU: float = 0.05

# Fallback distance threshold (metres) when IoU check is inconclusive
DISTANCE_THRESHOLD: float = 3.0

# Default parking spot dimensions (metres)
SPOT_DEPTH: float = 5.5
SPOT_WIDTH: float = 2.7

# Vehicle bounding box dimensions (metres) keyed by vehicle type name
VEHICLE_DIMS = {
    "truck":      (6.0, 2.5),   # (length, width)
    "suv":        (4.5, 2.0),
    "motorcycle": (2.2, 0.8),
    "qcar":       (0.4, 0.2),   # scaled QCar2
    "":           (4.0, 1.8),   # generic fallback
}


# ── Bounding box helpers ──────────────────────────────────────────────────────

def _vehicle_bbox(cx: float, cy: float, yaw_deg: float,
                  length: float, width: float) -> Tuple[float, float, float, float]:
    """
    Return axis-aligned bounding box (x_min, y_min, x_max, y_max) for a
    vehicle centred at (cx, cy) with the given yaw and dimensions.

    Uses the rotated rectangle's AABB so the IoU check is conservative
    (slightly larger than the true vehicle footprint) which is fine for
    parking detection — false negatives are worse than false positives.
    """
    rad   = math.radians(yaw_deg)
    cos_a = abs(math.cos(rad))
    sin_a = abs(math.sin(rad))

    half_w = (length * cos_a + width  * sin_a) / 2
    half_h = (length * sin_a + width  * cos_a) / 2

    return (cx - half_w, cy - half_h, cx + half_w, cy + half_h)


def _spot_bbox(spot: dict) -> Tuple[float, float, float, float]:
    """
    Return axis-aligned bounding box for a parking spot dict.

    Spot dict must have:
        center  : [x, y, z]
        yaw_deg : float
    Optional:
        depth   : float  (defaults to SPOT_DEPTH)
        width   : float  (defaults to SPOT_WIDTH)
    """
    cx  = spot["center"][0]
    cy  = spot["center"][1]
    yaw = spot.get("yaw_deg", 0)
    d   = spot.get("depth", SPOT_DEPTH)
    w   = spot.get("width", SPOT_WIDTH)

    rad   = math.radians(yaw)
    cos_a = abs(math.cos(rad))
    sin_a = abs(math.sin(rad))

    half_w = (d * cos_a + w * sin_a) / 2
    half_h = (d * sin_a + w * cos_a) / 2

    return (cx - half_w, cy - half_h, cx + half_w, cy + half_h)


def bbox_iou(b1: Tuple[float, float, float, float],
             b2: Tuple[float, float, float, float]) -> float:
    """
    Compute IoU between two axis-aligned bounding boxes.

    Both boxes are (x_min, y_min, x_max, y_max).
    Returns float in [0, 1].
    """
    ix1 = max(b1[0], b2[0])
    iy1 = max(b1[1], b2[1])
    ix2 = min(b1[2], b2[2])
    iy2 = min(b1[3], b2[3])

    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter == 0:
        return 0.0

    area1 = max(0.0, b1[2]-b1[0]) * max(0.0, b1[3]-b1[1])
    area2 = max(0.0, b2[2]-b2[0]) * max(0.0, b2[3]-b2[1])
    union = area1 + area2 - inter

    return inter / union if union > 0 else 0.0


# ── Per-vehicle IoU helpers ───────────────────────────────────────────────────

def spot_vehicle_iou(spot: dict, vx: float, vy: float,
                     vehicle_name: str = "",
                     vehicle_yaw: float = 0.0) -> float:
    """
    Compute IoU between a parking spot and a vehicle at world position (vx, vy).

    Parameters
    ----------
    spot         : Parking spot dict with center, yaw_deg, depth, width.
    vx, vy       : Vehicle centre in world coordinates.
    vehicle_name : Used to look up VEHICLE_DIMS.  Empty string = generic.
    vehicle_yaw  : Vehicle heading in degrees (0 if unknown).
    """
    name = vehicle_name.lower().strip()
    dims = VEHICLE_DIMS.get(name, VEHICLE_DIMS[""])
    v_bbox = _vehicle_bbox(vx, vy, vehicle_yaw, dims[0], dims[1])
    s_bbox = _spot_bbox(spot)
    return bbox_iou(s_bbox, v_bbox)


def spot_vehicle_iou_legacy(cx: float, cy: float,
                             vx: float, vy: float,
                             vehicle_name: str = "") -> float:
    """
    Backward-compatible wrapper: compute IoU from raw spot centre coordinates
    rather than a full spot dict.  Used by drone_scanner.py QCar checks.

    Assumes spot faces 0° (no yaw) and default dimensions.
    """
    mock_spot = {
        "center":  [cx, cy, 0.0],
        "yaw_deg": 0,
        "depth":   SPOT_DEPTH,
        "width":   SPOT_WIDTH,
    }
    return spot_vehicle_iou(mock_spot, vx, vy, vehicle_name)


# ── Main occupancy predicates ─────────────────────────────────────────────────

def is_spot_occupied_iou(spot: dict, vehicle_list: list,
                          iou_thresh: float = IOU_THRESHOLD) -> Tuple[bool, float]:
    """
    Return (occupied, best_iou) using pure IoU matching.

    vehicle_list entries can be:
        [x, y]          — generic vehicle, unknown dims
        [x, y, name]    — named vehicle, looks up VEHICLE_DIMS
        [x, y, name, yaw_deg]
    """
    best = 0.0
    for entry in vehicle_list:
        vx, vy = entry[0], entry[1]
        name   = entry[2] if len(entry) > 2 else ""
        yaw    = entry[3] if len(entry) > 3 else 0.0
        iou    = spot_vehicle_iou(spot, vx, vy, name, yaw)
        if iou > best:
            best = iou
    return best >= iou_thresh, best


def is_spot_occupied(spot: dict, vehicle_list: list,
                     iou_thresh: float   = IOU_THRESHOLD,
                     dist_thresh: float  = DISTANCE_THRESHOLD
                     ) -> Tuple[bool, float, str]:
    """
    Return (occupied, best_score, method) where method is 'iou' or 'distance'.

    Checks IoU first.  Falls back to Euclidean distance if no vehicle
    exceeds the IoU threshold — catches vehicles with slight offsets.

    Parameters
    ----------
    spot         : Parking spot dict.
    vehicle_list : List of [x, y] or [x, y, name] or [x, y, name, yaw].
    iou_thresh   : Minimum IoU to call a spot occupied.
    dist_thresh  : Fallback distance threshold in metres.

    Returns
    -------
    (occupied: bool, best_score: float, method: str)
    """
    occupied_iou, best_iou = is_spot_occupied_iou(spot, vehicle_list, iou_thresh)
    if occupied_iou:
        return True, best_iou, "iou"

    # Distance fallback
    cx = spot["center"][0]
    cy = spot["center"][1]
    for entry in vehicle_list:
        vx, vy = entry[0], entry[1]
        dist = math.hypot(vx - cx, vy - cy)
        if dist < dist_thresh:
            return True, dist, "distance"

    return False, best_iou, "iou"


def has_spillover(spot: dict, vehicle_list: list,
                  iou_thresh: float = SPILLOVER_IOU) -> Tuple[bool, float]:
    """
    Check whether a large vehicle from an adjacent spot spills into this one.

    Uses the lower SPILLOVER_IOU threshold so partial overlaps from
    trucks / SUVs straddling two spaces are detected.

    Returns (spillover: bool, best_iou: float).
    """
    occupied, best_iou = is_spot_occupied_iou(spot, vehicle_list, iou_thresh)
    return occupied, best_iou


# ── High-level API used by main.py ────────────────────────────────────────────

def get_vehicle_positions(qlabs, vehicle_actors: list) -> list:
    """
    Query QLabs for the current world position of each static vehicle actor.

    Parameters
    ----------
    qlabs          : Active QLabs connection.
    vehicle_actors : List of (actor_number, name, x, y) tuples as returned
                     by setup() in main.py.

    Returns
    -------
    List of [x, y, name] for each vehicle.
    """
    positions = []
    for actor_num, name, x, y in vehicle_actors:
        # Static vehicles don't move — use the spawn positions directly.
        # If you need live positions, call actor.get_world_transform() here.
        positions.append([x, y, name])
    return positions


def check_parking_spot_occupancy(qlabs, parking_spots: list,
                                  vehicle_positions: list = None) -> dict:
    """
    Check occupancy for every parking spot.

    Parameters
    ----------
    qlabs             : Active QLabs connection (kept for API compatibility).
    parking_spots     : List of spot dicts from build_parking_lot().
    vehicle_positions : List of [x, y, name] from get_vehicle_positions().
                        If None, all spots are marked unoccupied.

    Returns
    -------
    Dict mapping spot label → bool (True = occupied).
    """
    if vehicle_positions is None:
        vehicle_positions = []

    occupancy = {}
    for spot in parking_spots:
        label = spot.get("label", str(id(spot)))
        occupied, _, _ = is_spot_occupied(spot, vehicle_positions)
        occupancy[label] = occupied

    return occupancy


def get_empty_parking_spots(parking_spots: list, occupancy: dict) -> list:
    """
    Return list of spot dicts that are not occupied.

    Parameters
    ----------
    parking_spots : Full list of spot dicts.
    occupancy     : Dict of label → bool from check_parking_spot_occupancy().
    """
    return [
        spot for spot in parking_spots
        if not occupancy.get(spot.get("label", ""), False)
    ]


def print_occupancy_report(parking_spots: list, occupancy: dict):
    """
    Print a formatted occupancy report to stdout.
    """
    total    = len(parking_spots)
    occ_count = sum(1 for v in occupancy.values() if v)
    avail    = total - occ_count

    print("\n" + "="*50)
    print("PARKING LOT OCCUPANCY REPORT")
    print("="*50)
    print(f"  Total spaces : {total}")
    print(f"  Occupied     : {occ_count}")
    print(f"  Available    : {avail}")

    occupied_labels = sorted(
        label for label, occ in occupancy.items() if occ
    )
    if occupied_labels:
        print(f"\n  Occupied spots: {', '.join(occupied_labels)}")
    else:
        print("\n  No occupied spots.")
    print("="*50 + "\n")
