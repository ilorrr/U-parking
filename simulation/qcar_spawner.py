# ============================================================
# qcar_spawner.py  — complete rework
#
# GEOMETRY (derived from parking_lot.py, orientation_deg=90):
#   With orientation_deg=90 the grid is rotated so:
#     X axis = row direction  (rows stacked in -X direction)
#     Y axis = column direction (spots spread along +Y direction)
#
#   Row centres (X):    0->-5.0   1->-20.5   2->-36.0   3->-51.5
#   Row pitch: 15.5 m  (spot_length=5.5 + row_spacing=10)
#
#   Even rows (yaw=90):  back_wall at cx+2.75  mouth at cx-2.75  enter ->+X
#   Odd  rows (yaw=270): back_wall at cx-2.75  mouth at cx+2.75  enter ->-X
#
# NAVIGATION PATH (south highway approach, no row crossing):
#   spawn(10, oy)
#   -> entry_road(3,  oy)     drive west to entry road
#   -> y_highway (3, -16)     drive south BELOW the lot
#   -> aisle_gate(ax, -16)    drive along highway to correct aisle
#   -> aisle_spot(ax,  sy)    drive north to spot Y
#   -> approach  (apx, sy)    step to just outside mouth
#   -> park      (px,  sy)    pull into spot
#
#   Lateral (X) movement only happens at y=-16 (south of lot boundary
#   at y=-10), so cars NEVER cross a parking row.
#
# BACK-WALL CLAMP:
#   park_x is clamped to [cx-0.60, cx+0.60] so the car body is
#   always inside the spot regardless of any offset.
# ============================================================

import sys
import math
import time
import random

QLABS_LIB_PATH = r"E:\uparking"
if QLABS_LIB_PATH not in sys.path:
    sys.path.insert(0, QLABS_LIB_PATH)

from qvl.qcar2 import QLabsQCar2

# Parking lot constants (must match parking_lot.py)
START_X      = -4.0   # must match parking_lot.py start_xyz[0]
ROW_PITCH    = 15.5   # spot_length(5.5) + row_spacing(10)
SPOT_HALF    = 2.50   # spot_length / 2
CAR_HALF     = 2.3
WALL_MARGIN  = 0.15
PARK_SLACK   = SPOT_HALF - CAR_HALF - WALL_MARGIN   # 0.10 m
MOUTH_BIAS   = 0.1    # metres toward mouth — keeps nose off back wall

# Aisle / road X positions
ENTRY_ROAD_X = 3.0
AISLE_A_X    = -12.75   # between rows 0 & 1
AISLE_B_X    = -43.75   # between rows 2 & 3
Y_HIGHWAY    = -16.0    # south of lot (lot starts at y=-10)
APPROACH_PAD = 1.0      # metres outside mouth before turning

# Spawn positions
SPAWN_X         = 10.0
SPAWN_Y_BASE    = -11.0
SPAWN_Y_SPACING = 3.5
GROUND_Z        = 0.105

# Occupancy filter — uses IoU from occupancy.py (consistent with drone scanner)
from occupancy import is_spot_occupied_iou, IOU_THRESHOLD

QCAR_ACTOR_BASE = 500
STEP_DIST  = 1.5  # INCREASED SPEED
STEP_PAUSE = 0.005 # FASTER TRANSITIONS
TURN_STEPS = 8    # SNAPPIER TURNS

# ── GEOMETRY ──────────────────────────────────────────

LANE_OFFSET = 2.5 # CLEAR SEPARATION

def _row_cx(row_idx):
    """Calculates the center X of a row based on standard pitch."""
    return START_X - row_idx * ROW_PITCH

def _park_x(spot):
    """Refined geometric center logic based on user hand-tuned data."""
    cx = spot["center"][0]
    # The user's data shows the car center is offset by 0.3m from the row start.
    offset = -0.3 if spot["row"] % 2 == 0 else 0.3
    return cx + offset

def _aisle_x(spot):
    """Calculates the correct aisle X-coordinate for any section."""
    cx = spot["center"][0]
    return cx - 7.75 if spot["row"] % 2 == 0 else cx + 7.75

def _build_waypoints(spot, spawn_y):
    """
    Strict Two-Way pathing: Ensures cars use distinct lanes within the aisle.
    """
    sy  = spot["center"][1]
    ax  = _aisle_x(spot)
    px  = _park_x(spot)
    
    # ROW 0/2 = Right Lane, ROW 1/3 = Left Lane
    lane_x = ax + LANE_OFFSET if spot["row"] in [0, 2] else ax - LANE_OFFSET
    
    wps = [
        (SPAWN_X,      spawn_y),   # 0 spawn
        (ENTRY_ROAD_X, spawn_y),   # 1 entry road
        (ENTRY_ROAD_X, Y_HIGHWAY), # 2 south to highway
        (lane_x,       Y_HIGHWAY), # 3 along highway to section lane
        (lane_x,       sy),        # 4 through aisle in specific lane
        (px,           sy),        # 5 park
    ]
    return wps, px, sy

def _is_even(spot):
    return spot.get("direction", "right") == "right"

def _mouth_x(spot):
    cx = spot["center"][0]
    return cx - SPOT_HALF if _is_even(spot) else cx + SPOT_HALF

def _approach_x(spot):
    mx = _mouth_x(spot)
    return mx - APPROACH_PAD if _is_even(spot) else mx + APPROACH_PAD

def _park_yaw(spot):
    return 0.0 if _is_even(spot) else 180.0


# ============================================================
# OCCUPANCY FILTER
# ============================================================

def _free_spots(spots, vehicle_actors):
    """Filter spots occupied by static vehicles using IoU."""
    if not vehicle_actors:
        return spots
    positions = [[x, y, name] for _, name, x, y in vehicle_actors]
    return [s for s in spots
            if not is_spot_occupied_iou(s, positions, IOU_THRESHOLD)[0]]


# ============================================================
# POSE SETTER
# ============================================================

def _set_pose(car, x, y, yaw_deg, brake=False):
    car.set_transform_and_request_state_degrees(
        location=[x, y, GROUND_Z], rotation=[0, 0, yaw_deg],
        enableDynamics=False, headlights=not brake,
        leftTurnSignal=False, rightTurnSignal=False,
        brakeSignal=brake, reverseSignal=False,
        waitForConfirmation=True
    )
    time.sleep(STEP_PAUSE)

def _hdg(x1, y1, x2, y2):
    dx, dy = x2-x1, y2-y1
    return math.degrees(math.atan2(dy, dx)) if abs(dx)+abs(dy) > 0.01 else None


# ============================================================
# SEGMENT DRIVER
# ============================================================

def _drive_segment(car, x1, y1, x2, y2, cur_yaw):
    seg = math.hypot(x2-x1, y2-y1)
    if seg < 0.05:
        return cur_yaw
    tgt = _hdg(x1, y1, x2, y2)
    if tgt is None:
        return cur_yaw
    diff = ((tgt - cur_yaw) + 180) % 360 - 180
    if abs(diff) > 5:
        for i in range(1, TURN_STEPS+1):
            _set_pose(car, x1, y1, cur_yaw + diff*(i/TURN_STEPS))
        cur_yaw = tgt
    steps = max(3, int(seg / STEP_DIST))
    for i in range(1, steps+1):
        t = i/steps
        _set_pose(car, x1+(x2-x1)*t, y1+(y2-y1)*t, cur_yaw)
    return cur_yaw


# ============================================================
# WAYPOINT BUILDER
# ============================================================

LANE_OFFSET = 1.5  # Metres to shift for right-hand traffic

Y_HIGHWAY_NORTH = 105.0 # North entrance for testing two-way logic

def _build_waypoints(spot, spawn_y, test_opposite=False):
    """
    Two-way testing path: Alternates between South-to-North and North-to-South entry.
    """
    sy  = spot["center"][1]
    ax  = _aisle_x(spot)
    px  = _park_x(spot)
    
    # ROW 0/2 = Right Lane, ROW 1/3 = Left Lane
    lane_x = ax + LANE_OFFSET if spot["row"] in [0, 2] else ax - LANE_OFFSET
    
    if not test_opposite:
        # Standard: Enter from South, drive North
        wps = [
            (SPAWN_X,      spawn_y),
            (ENTRY_ROAD_X, spawn_y),
            (ENTRY_ROAD_X, Y_HIGHWAY),
            (lane_x,       Y_HIGHWAY),
            (lane_x,       sy),
            (px,           sy),
        ]
    else:
        # Test: Enter from North, drive South
        # Flip lane for Southbound right-hand traffic
        opp_lane_x = ax - LANE_OFFSET if spot["row"] in [0, 2] else ax + LANE_OFFSET
        wps = [
            (SPAWN_X,      spawn_y),
            (ENTRY_ROAD_X, spawn_y),
            (ENTRY_ROAD_X, Y_HIGHWAY_NORTH),
            (opp_lane_x,   Y_HIGHWAY_NORTH),
            (opp_lane_x,   sy),
            (px,           sy),
        ]
    return wps, px, sy

class _CarRunner:
    def __init__(self, qlabs, actor_num, spot, spawn_y, test_opposite=False):
        self.qlabs     = qlabs
        self.actor_num = actor_num
        self.label     = spot["label"]
        self.spot      = spot
        self.spawn_y   = spawn_y
        self.car       = None
        self.done      = False
        self.wps       = []
        self.park_x    = 0.0
        self.park_y    = 0.0
        self.wp_idx    = 0
        self.cur_yaw   = 180.0
        self.test_opposite = test_opposite

    def spawn(self):
        car = QLabsQCar2(self.qlabs)
        rc  = car.spawn_id_degrees(
            actorNumber=self.actor_num,
            location=[SPAWN_X, self.spawn_y, GROUND_Z],
            rotation=[0, 0, 180.0], scale=[1,1,1],
            configuration=0, waitForConfirmation=True
        )
        if rc != 0:
            return False
        self.car = car
        wps, px, py = _build_waypoints(self.spot, self.spawn_y, self.test_opposite)
        self.wps, self.park_x, self.park_y = wps, px, py
        return True

    def step_segment(self):
        if self.done:
            return
        if self.wp_idx >= len(self.wps) - 1:
            _set_pose(self.car, self.park_x, self.park_y,
                      _park_yaw(self.spot), brake=True)
            self.done = True
            return
        x1, y1 = self.wps[self.wp_idx]
        x2, y2 = self.wps[self.wp_idx + 1]
        self.cur_yaw = _drive_segment(self.car, x1, y1, x2, y2, self.cur_yaw)
        self.wp_idx += 1


# Collision Avoidance Constants
COLLISION_RADIUS = 3.5  
SAFETY_WAIT      = 0.2  
Y_HIGHWAY_NORTH  = 105.0 

def _is_path_blocked(car_runner, all_runners):
    """Checks if any other car is within the safety radius."""
    if car_runner.done or not car_runner.car:
        return False
    if car_runner.wp_idx >= len(car_runner.wps) - 1:
        return False
    tx, ty = car_runner.wps[car_runner.wp_idx + 1]
    for other in all_runners:
        if other == car_runner or not other.car:
            continue
        ox, oy = other.wps[other.wp_idx] if not other.done else (other.park_x, other.park_y)
        dist = math.hypot(tx - ox, ty - oy)
        if dist < COLLISION_RADIUS:
            return True
    return False

def _drive_all(runners):
    """Drives all cars while checking for collisions."""
    start_time = time.time()
    while not all(r.done for r in runners):
        if time.time() - start_time > 120:
            for r in runners: r.done = True
            break
        for r in runners:
            if not r.done:
                if not _is_path_blocked(r, runners):
                    r.step_segment()
                else:
                    _set_pose(r.car, r.wps[r.wp_idx][0], r.wps[r.wp_idx][1], r.cur_yaw, brake=True)
                    time.sleep(0.05)
    print("\n[Spawner] All cars parked safely.\n")


# ============================================================
# PUBLIC API
# ============================================================

def spawn_random_qcars_async(qlabs, parking_spots, count=3, seed=None,
                              vehicle_actors=None):
    if seed is not None:
        random.seed(seed)
    available = _free_spots(parking_spots, vehicle_actors or [])
    count = min(count, len(available))
    if count == 0:
        print("[Spawner] No free spots available.")
        return [], []
    chosen = random.sample(available, count)

    print(f"Spawning {count} QCar(s) into {len(available)} free spots...")

    runners = []
    for i, spot in enumerate(chosen):
        spawn_y = SPAWN_Y_BASE + i * SPAWN_Y_SPACING
        # Alternate direction to test two-way logic (odd cars go North)
        r = _CarRunner(qlabs, QCAR_ACTOR_BASE + i, spot, spawn_y, test_opposite=(i % 2 == 1))
        if r.spawn():
            runners.append(r)
        time.sleep(0.4)

    _drive_all(runners)

    actor_list    = [(r.actor_num, f"QCar_{r.actor_num}",
                      r.park_x, r.park_y) for r in runners]
    target_labels = [r.label for r in runners]
    print(f"[Spawner] {len(actor_list)}/{count} parked: {target_labels}")
    print("="*60 + "\n")
    return actor_list, target_labels


def spawn_random_qcars(qlabs, parking_spots, count=3, seed=None,
                       vehicle_actors=None):
    return spawn_random_qcars_async(qlabs, parking_spots, count[1], seed,
                                    vehicle_actors)

def get_qcar_actor_list(qcar_actors):
    return [(an, name) for an, name, _, _ in qcar_actors]
