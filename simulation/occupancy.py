# ============================================================
# occupancy.py
# IoU-based parking spot occupancy detection.
#
# Replaces the old fixed bounding-box proximity check with
# Intersection over Union (IoU) — a proper overlap metric:
#
#   IoU = intersection_area / union_area
#
# A spot is OCCUPIED when IoU >= IOU_THRESHOLD (default 0.10).
#
# WHY IoU IS BETTER:
#   Old method: "is the vehicle *centre* within ±2.4m × ±0.6m?"
#     → misses vehicles parked off-centre, catches vehicles
#       whose centre passes nearby but body doesn't overlap
#   IoU method: "do the vehicle and spot rectangles actually overlap
#               by at least 10% of their combined area?"
#     → correct regardless of where the vehicle centre sits,
#       works for all vehicle sizes including motorcycles
#
# VEHICLE DIMENSIONS (matched to vehicles.py spawn sizes):
#   QCar2      : 5.0m × 2.0m
#   Truck      : 5.2m × 2.3m
#   SUV        : 4.5m × 1.9m
#   Motorcycle : 2.2m × 0.8m
#
# SPOT DIMENSIONS (matched to parking_lot.py):
#   Depth (X axis) : 5.5m
#   Width (Y axis) : 2.7m
# ============================================================

from collections import defaultdict


# ── IoU configuration ─────────────────────────────────────────

IOU_THRESHOLD = 0.10   # minimum overlap fraction to classify as occupied
                        # 0.10 catches all types including motorcycles (IoU~0.12)
                        # raise to 0.20 to reduce false positives from aisle traffic

# Parking spot dimensions (must match parking_lot.py)
SPOT_DEPTH = 5.5
SPOT_WIDTH = 2.7

# Vehicle half-dimensions (half_length, half_width)
# Keyed by lowercase substring of the vehicle name
VEHICLE_DIMS = {
    "qcar":       (2.50, 1.00),
    "truck":      (2.60, 1.15),
    "suv":        (2.25, 0.95),
    "motorcycle": (1.10, 0.40),
    "default":    (2.50, 1.00),   # fallback
}


# ============================================================
# CORE IoU FUNCTIONS
# ============================================================

def bbox_iou(cx1, cy1, w1, h1, cx2, cy2, w2, h2):
    """
    Intersection over Union for two axis-aligned bounding boxes.
    Boxes specified as (centre_x, centre_y, full_width, full_height).
    Returns float in [0.0, 1.0].
    """
    x_overlap = max(0.0, min(cx1 + w1/2, cx2 + w2/2)
                        - max(cx1 - w1/2, cx2 - w2/2))
    y_overlap = max(0.0, min(cy1 + h1/2, cy2 + h2/2)
                        - max(cy1 - h1/2, cy2 - h2/2))
    intersection = x_overlap * y_overlap
    union = w1 * h1 + w2 * h2 - intersection
    return intersection / union if union > 0.0 else 0.0


def spot_vehicle_iou(spot_cx, spot_cy, vehicle_cx, vehicle_cy,
                     vehicle_name=""):
    """
    IoU between a parking spot and a specific vehicle.
    Looks up vehicle dimensions by name; falls back to QCar size.
    """
    name_lower = vehicle_name.lower()
    half_l, half_w = VEHICLE_DIMS["default"]
    for key, dims in VEHICLE_DIMS.items():
        if key != "default" and key in name_lower:
            half_l, half_w = dims
            break

    return bbox_iou(
        spot_cx,    spot_cy,    SPOT_DEPTH,  SPOT_WIDTH,
        vehicle_cx, vehicle_cy, half_l * 2,  half_w * 2
    )


def is_spot_occupied_iou(spot, vehicle_positions,
                         threshold=IOU_THRESHOLD):
    """
    Return (occupied: bool, best_iou: float) for a single spot.

    Args:
        spot              : spot dict with "center" key
        vehicle_positions : list of [x, y] or [x, y, name]
        threshold         : IoU threshold

    Returns:
        (bool, float)
    """
    cx, cy  = spot["center"][0], spot["center"][1]
    best    = 0.0

    for entry in vehicle_positions:
        vx, vy  = entry[0], entry[1]
        vname   = str(entry[2]) if len(entry) >= 3 else ""
        score   = spot_vehicle_iou(cx, cy, vx, vy, vname)
        if score > best:
            best = score

    return best >= threshold, round(best, 4)


# ============================================================
# PUBLIC API
# ============================================================

def get_vehicle_positions(qlabs, vehicle_actors):
    """
    Extract [x, y, name] from vehicle_actors tuples.
    Name is passed through so IoU can look up correct dimensions.
    """
    return [[x, y, name] for _, name, x, y in vehicle_actors]


def check_parking_spot_occupancy(qlabs, parking_spots,
                                 vehicle_positions=None):
    """
    Check every spot using IoU. Returns dict label -> bool.
    """
    vehicle_positions = vehicle_positions or []
    return {
        spot.get("label", str(spot.get("spot_id", -1))):
            is_spot_occupied_iou(spot, vehicle_positions)[0]
        for spot in parking_spots
    }


def get_empty_parking_spots(parking_spots, occupancy_status):
    return [
        spot for spot in parking_spots
        if not occupancy_status.get(
            spot.get("label", str(spot.get("spot_id", -1))), False)
    ]


def print_occupancy_report(spots, occupancy):
    pass   # output handled in main.py
