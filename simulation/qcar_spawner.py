# ============================================================
# qcar_spawner.py  - complete rework
# ============================================================

import sys
import math
import time
import random
import os

# Fix: Updated QLabs library path for jairD
QLABS_LIB_PATH = r"C:\Users\jairD\OneDrive\Documents\U-Parking\0_libraries\python"
if QLABS_LIB_PATH not in sys.path:
    sys.path.insert(0, QLABS_LIB_PATH)

from qvl.qcar2 import QLabsQCar2

# Parking lot constants (must match parking_lot.py)
START_X      = -4.0   
ROW_PITCH    = 15.5   
SPOT_HALF    = 2.50   
CAR_HALF     = 2.3
WALL_MARGIN  = 0.15
PARK_SLACK   = SPOT_HALF - CAR_HALF - WALL_MARGIN   # 0.10 m
MOUTH_BIAS   = 0.1    # metres toward mouth

# Aisle / road X positions
ENTRY_ROAD_X = 3.0
AISLE_A_X    = -12.75   
AISLE_B_X    = -43.75   
Y_HIGHWAY    = -16.0    
APPROACH_PAD = 1.0      

# Spawn positions
SPAWN_X         = 10.0
SPAWN_Y_BASE    = -11.0
SPAWN_Y_SPACING = 3.5
GROUND_Z        = 0.105

from occupancy import is_spot_occupied_iou, IOU_THRESHOLD

QCAR_ACTOR_BASE = 500
STEP_DIST  = 1.5  
STEP_PAUSE = 0.005 
TURN_STEPS = 8    

LANE_OFFSET = 2.5 

def _row_cx(row_idx):
    return START_X - row_idx * ROW_PITCH

def _park_x(spot):
    cx = spot["center"][0]
    offset = -0.3 if spot["row"] % 2 == 0 else 0.3
    return cx + offset

def _aisle_x(spot):
    cx = spot["center"][0]
    return cx - 7.75 if spot["row"] % 2 == 0 else cx + 7.75

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

def _free_spots(spots, vehicle_actors):
    if not vehicle_actors:
        return spots
    positions = [[x, y, name] for _, name, x, y in vehicle_actors]
    return [s for s in spots
            if not is_spot_occupied_iou(s, positions, IOU_THRESHOLD)[0]]

ROW_PRIORITY = {'A': 0, 'B': 1, 'C': 2, 'D': 3}
AISLE_GROUP  = {'A': 0, 'B': 0, 'C': 1, 'D': 1}

def _parse_label(label):
    core = label.split('-')[-1]   
    row_letter = core[0]
    try:
        col = int(core[1:])
    except ValueError:
        col = 999
    return row_letter, col

def _pick_spots_by_row(free_spots, count):
    from collections import defaultdict
    rows = defaultdict(list)
    for s in free_spots:
        row_letter, col = _parse_label(s["label"])
        rows[row_letter].append((col, s))
    for letter in rows:
        rows[letter].sort(key=lambda x: x[0])
    row_order = sorted(rows.keys(), key=lambda r: ROW_PRIORITY.get(r, 99))
    chosen = []
    row_idx = [0] * len(row_order)
    while len(chosen) < count:
        added_any = False
        for i, letter in enumerate(row_order):
            if len(chosen) >= count:
                break
            if row_idx[i] < len(rows[letter]):
                _, spot = rows[letter][row_idx[i]]
                chosen.append(spot)
                row_idx[i] += 1
                added_any = True
        if not added_any:
            break   
    return chosen

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

Y_HIGHWAY_NORTH = 105.0  

def _build_waypoints(spot, spawn_y, test_opposite=False):
    sy  = spot["center"][1]
    ax  = _aisle_x(spot)
    px  = _park_x(spot)
    lane_x = ax + LANE_OFFSET if spot["row"] in [0, 2] else ax - LANE_OFFSET
    if not test_opposite:
        wps = [
            (SPAWN_X,      spawn_y),
            (ENTRY_ROAD_X, spawn_y),
            (ENTRY_ROAD_X, Y_HIGHWAY),
            (lane_x,       Y_HIGHWAY),
            (lane_x,       sy),
            (px,           sy),
        ]
    else:
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

COLLISION_RADIUS = 3.5   
FOLLOW_RADIUS    = 2.5   
SAFETY_WAIT      = 0.2

def _heading(runner):
    if runner.wp_idx >= len(runner.wps) - 1:
        return (0, 0)
    x1, y1 = runner.wps[runner.wp_idx]
    x2, y2 = runner.wps[runner.wp_idx + 1]
    dist = math.hypot(x2 - x1, y2 - y1)
    if dist < 0.01:
        return (0, 0)
    return ((x2 - x1) / dist, (y2 - y1) / dist)

def _is_path_blocked(car_runner, all_runners):
    if car_runner.done or not car_runner.car:
        return False
    if car_runner.wp_idx >= len(car_runner.wps) - 1:
        return False
    tx, ty = car_runner.wps[car_runner.wp_idx + 1]
    my_hdg = _heading(car_runner)
    for other in all_runners:
        if other == car_runner or not other.car or other.done:
            continue
        ox, oy = (other.wps[other.wp_idx] if other.wp_idx < len(other.wps)
                  else (other.park_x, other.park_y))
        dist = math.hypot(tx - ox, ty - oy)
        other_hdg = _heading(other)
        dot = my_hdg[0] * other_hdg[0] + my_hdg[1] * other_hdg[1]
        if dot < -0.3:
            if dist < COLLISION_RADIUS:
                return True
        else:
            to_other_x = ox - tx
            to_other_y = oy - ty
            ahead = my_hdg[0] * to_other_x + my_hdg[1] * to_other_y
            if ahead > 0 and dist < FOLLOW_RADIUS:
                return True
    return False

def _drive_all(runners):
    start_time = time.time()
    while not all(r.done for r in runners):
        if time.time() - start_time > 180:  
            for r in runners:
                r.done = True
            break
        for r in runners:
            if not r.done:
                if not _is_path_blocked(r, runners):
                    r.step_segment()
                else:
                    _set_pose(r.car, r.wps[r.wp_idx][0],
                              r.wps[r.wp_idx][1], r.cur_yaw, brake=True)
                    time.sleep(SAFETY_WAIT)

def spawn_random_qcars_async(qlabs, parking_spots, count=3, seed=None,
                              vehicle_actors=None):
    available = _free_spots(parking_spots, vehicle_actors or [])
    count = min(count, len(available))
    if count == 0:
        return [], []
    car_positions = [
        (SPAWN_X, SPAWN_Y_BASE + i * SPAWN_Y_SPACING)
        for i in range(count)
    ]
    from parking_assignment import assign_spots_hungarian, compute_assignment_metrics
    chosen = assign_spots_hungarian(car_positions, available)
    runners = []
    for i, spot in enumerate(chosen):
        spawn_y = car_positions[i][1]
        r = _CarRunner(qlabs, QCAR_ACTOR_BASE + i, spot, spawn_y, test_opposite=(i % 2 == 1))
        if r.spawn():
            runners.append(r)
        time.sleep(0.4)
    _drive_all(runners)
    actor_list    = [(r.actor_num, f"QCar_{r.actor_num}",
                      r.park_x, r.park_y) for r in runners]
    target_labels = [r.label for r in runners]
    return actor_list, target_labels

def spawn_random_qcars(qlabs, parking_spots, count=3, seed=None,
                       vehicle_actors=None):
    return spawn_random_qcars_async(qlabs, parking_spots, count, seed,
                                    vehicle_actors)

def get_qcar_actor_list(qcar_actors):
    return [(an, name) for an, name, _, _ in qcar_actors]
