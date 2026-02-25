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
SPAWN_Y_BASE    = -20.0
SPAWN_Y_SPACING = 6.0
GROUND_Z        = 0.005

# Occupancy filter — uses IoU from occupancy.py (consistent with drone scanner)
from occupancy import is_spot_occupied_iou, IOU_THRESHOLD

QCAR_ACTOR_BASE = 500
STEP_DIST  = 0.8
STEP_PAUSE = 0.01
TURN_STEPS = 10


# ============================================================
# GEOMETRY
# ============================================================

def _row_cx(row_idx):
    return START_X - row_idx * ROW_PITCH

def _is_even(spot):
    return spot.get("direction", "right") == "right"

def _mouth_x(spot):
    cx = _row_cx(spot["row"])
    return cx - SPOT_HALF if _is_even(spot) else cx + SPOT_HALF

def _approach_x(spot):
    mx = _mouth_x(spot)
    return mx - APPROACH_PAD if _is_even(spot) else mx + APPROACH_PAD

def _park_x(spot):
    cx      = _row_cx(spot["row"])
    bias    = -MOUTH_BIAS if _is_even(spot) else MOUTH_BIAS
    desired = cx + bias
    return max(cx - PARK_SLACK, min(desired, cx + PARK_SLACK))

def _aisle_x(spot):
    return AISLE_A_X if spot["row"] in (0, 1) else AISLE_B_X

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

def _build_waypoints(spot, spawn_y):
    """
    Six-segment path that keeps lateral movement south of the lot.
    Cars never cross a parking row.
    """
    sy  = spot["center"][1]
    ax  = _aisle_x(spot)
    apx = _approach_x(spot)
    px  = _park_x(spot)
    wps = [
        (SPAWN_X,      spawn_y),   # 0 spawn
        (ENTRY_ROAD_X, spawn_y),   # 1 entry road
        (ENTRY_ROAD_X, Y_HIGHWAY), # 2 south to highway
        (ax,           Y_HIGHWAY), # 3 along highway to aisle
        (ax,           sy),        # 4 north to spot Y
        (apx,          sy),        # 5 approach mouth
        (px,           sy),        # 6 park
    ]
    return wps, px, sy


# ============================================================
# CAR RUNNER
# ============================================================

class _CarRunner:
    def __init__(self, qlabs, actor_num, spot, spawn_y):
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

    def spawn(self):
        car = QLabsQCar2(self.qlabs)
        rc  = car.spawn_id_degrees(
            actorNumber=self.actor_num,
            location=[SPAWN_X, self.spawn_y, GROUND_Z],
            rotation=[0, 0, 180.0], scale=[1,1,1],
            configuration=0, waitForConfirmation=True
        )
        if rc != 0:
            print(f"  [QCar {self.actor_num}] spawn FAILED (rc={rc})")
            self.done = True
            return False
        self.car = car
        wps, px, py = _build_waypoints(self.spot, self.spawn_y)
        self.wps, self.park_x, self.park_y = wps, px, py
        row   = self.spot["row"]
        aisle = "A" if row in (0,1) else "B"
        side  = "even" if _is_even(self.spot) else "odd"
        cx    = _row_cx(row)

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


# ============================================================
# INTERLEAVED DRIVER
# ============================================================

def _drive_all(runners):
    while not all(r.done for r in runners):
        for r in runners:
            if not r.done:
                r.step_segment()
    print("\n[Spawner] All cars parked.\n")


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
        r = _CarRunner(qlabs, QCAR_ACTOR_BASE + i, spot, spawn_y)
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
