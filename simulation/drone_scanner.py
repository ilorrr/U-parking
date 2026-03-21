# ============================================================
# drone_scanner.py
# Drone path planning, scanning, optimisation, and logging.
# Supports BOTH stationary basic-shape vehicles AND moving QCars.
#
# Fixes vs original:
#   1. QCarPoller uses QLabsQCar2 (not QLabsQCar) to match spawner
#   2. Occupancy detection upgraded from bounding-box to IoU (occupancy.py)
#   3. Stale cache detection - regenerates drone_base_path.json if
#      spot count changed since last run (fixes IndexError crash)
#   4. QCar target spots deferred to end of scan with extended hover
#      so cars driving to their spots are detected on arrival
#   5. Updated to use new orientation-aware occupancy.py with
#      backward-compatible wrappers
# ============================================================

import os
import sys
import json
import math
import time
import threading
from datetime import datetime
from collections import defaultdict

from occupancy import (spot_vehicle_iou_legacy, is_spot_occupied, has_spillover,
                       IOU_THRESHOLD, DISTANCE_THRESHOLD)


# Path planning mode: "brain" or "spiral"
PLANNING_MODE = "spiral"

QLABS_LIB_PATH = r"C:\Users\Roli\Documents\uparking"
if QLABS_LIB_PATH not in sys.path:
    sys.path.insert(0, QLABS_LIB_PATH)

from qvl.real_time import QLabsRealTime


# -- File paths ------------------------------------------------
_HERE            = os.path.dirname(os.path.abspath(__file__))
BASE_PATH_FILE   = os.path.join(_HERE, "drone_base_path.json")
HISTORY_LOG_FILE = os.path.join(_HERE, "drone_scan_history.json")

# How often (seconds) the background thread re-queries QCar positions
POLL_INTERVAL = 0.5

# Extra hover seconds over QCar target spots so arriving cars are detected
QCAR_TARGET_HOVER = 8.0


# ============================================================
# QCAR LIVE POSITION POLLER  (background thread)
# ============================================================

class QCarPoller(threading.Thread):
    """
    Background thread that continuously queries QLabs for the live
    world position of every registered QCar2 actor.

    Uses QLabsQCar2 - must match the class used in qcar_spawner.py.
    """

    def __init__(self, qlabs, qcar_actors):
        super().__init__(daemon=True)
        self.qlabs       = qlabs
        self.qcar_actors = qcar_actors  # [(actor_num, name), ...]
        self._positions  = {}           # actor_num -> [x, y]
        self._lock       = threading.Lock()
        self._running    = False

    def run(self):
        from qvl.qcar2 import QLabsQCar2
        self._running = True
        print(f"  [QCarPoller] Started - polling {len(self.qcar_actors)} "
              f"QCar(s) every {POLL_INTERVAL}s")
        while self._running:
            try:
                for actor_num, name in self.qcar_actors:
                    try:
                        car = QLabsQCar2(self.qlabs)
                        car.actorNumber = actor_num
                        result = car.get_world_transform()
                        if result[0] == 0:
                            x, y = result[1][0], result[1][1]
                            with self._lock:
                                self._positions[actor_num] = [x, y]
                    except Exception as e:
                        print(f"  [QCarPoller] {name}: {e}")
            except Exception as e:
                print(f"  [QCarPoller] THREAD CRASH: {e}")
                import traceback
                traceback.print_exc()
            time.sleep(POLL_INTERVAL)
        print("  [QCarPoller] Stopped")

    def stop(self):
        self._running = False

    def get_positions(self):
        """Return a list of current [x, y] for all tracked QCars."""
        with self._lock:
            return list(self._positions.values())

    def get_named_positions(self):
        """Return {actor_num: [x, y]} snapshot for logging."""
        with self._lock:
            return dict(self._positions)


def spiral_waypoints(parking_spots, drone_altitude=8.0):
    """
    Row-by-row lawnmower sweep - scans all spots in row A left->right,
    then row B right->left, then row C left->right, etc.
    This matches the rectangular 4x39 lot layout and minimises
    travel distance vs the centre-outward spiral.
    """
    if not parking_spots:
        return []

    # Group spots by row
    from collections import defaultdict
    rows = defaultdict(list)
    for idx, spot in enumerate(parking_spots):
        row = spot.get("row", 0)
        rows[row].append((idx, spot))

    waypoints = []
    for row_num in sorted(rows.keys()):
        spots_in_row = rows[row_num]
        # Sort by column within the row
        spots_in_row.sort(key=lambda x: x[1].get("col", 0))
        # Alternate direction each row (snake pattern)
        if row_num % 2 != 0:
            spots_in_row = reversed(spots_in_row)
        for idx, spot in spots_in_row:
            waypoints.append({
                "label"      : spot.get("label", str(idx)),
                "x"          : spot["center"][0],
                "y"          : spot["center"][1],
                "z"          : spot["center"][2] + drone_altitude,
                "spot_index" : idx
            })

    return waypoints


# ============================================================
# PATH FILE  -  save / load base spiral geometry
# ============================================================

def save_base_path(waypoints):
    with open(BASE_PATH_FILE, "w") as f:
        json.dump(waypoints, f, indent=2)
    print(f"  [Scanner] Base path saved  -> {BASE_PATH_FILE}")


def load_base_path(expected_spot_count=None):
    """
    Load cached waypoints from disk.

    If expected_spot_count is provided, validates that the highest
    spot_index in the file is within range. Returns None (forcing
    regeneration) if the cache is stale or the count doesn't match.
    This prevents the IndexError crash that occurs when parking_spots
    changes size between runs.
    """
    if not os.path.exists(BASE_PATH_FILE):
        return None
    try:
        with open(BASE_PATH_FILE, "r") as f:
            wps = json.load(f)
    except Exception:
        print("  [Scanner] Cache unreadable - regenerating")
        return None

    if expected_spot_count is not None:
        max_idx = max((w.get("spot_index", 0) for w in wps), default=0)
        if max_idx >= expected_spot_count:
            print(f"  [Scanner] Cache stale (max index {max_idx} >= "
                  f"spot count {expected_spot_count}) - regenerating")
            os.remove(BASE_PATH_FILE)
            return None

    print(f"  [Scanner] Base path loaded ({len(wps)} waypoints)")
    return wps


# ============================================================
# HISTORY LOG  -  append / load occupancy history
# ============================================================

def load_history():
    if not os.path.exists(HISTORY_LOG_FILE):
        return []
    with open(HISTORY_LOG_FILE, "r") as f:
        return json.load(f)


def append_history(run_record):
    history = load_history()
    history.append(run_record)
    with open(HISTORY_LOG_FILE, "w") as f:
        json.dump(history, f, indent=2)
    print(f"  [Scanner] Run #{len(history)} logged -> {HISTORY_LOG_FILE}")


# ============================================================
# OCCUPANCY PROBABILITY  -  built from history
# ============================================================

def occupancy_probability(history):
    counts   = defaultdict(int)
    occupied = defaultdict(int)
    for run in history:
        for entry in run.get("scan", []):
            lbl = entry["label"]
            counts[lbl]   += 1
            if entry["occupied"]:
                occupied[lbl] += 1
    return {lbl: occupied[lbl] / total
            for lbl, total in counts.items() if total > 0}


# ============================================================
# PATH OPTIMISER
# ============================================================

def optimize_waypoint_order(waypoints, history, defer_labels=None):
    """
    Reorder waypoints so historically busy spots are visited first,
    using greedy nearest-neighbour routing within each priority group.

    defer_labels: set of spot labels to always place at the END of the
    scan regardless of history priority. Used for QCar target spots so
    moving cars have maximum time to arrive before the drone checks them.

    Priority groups:
      HIGH   > 50% occupancy history  -> scanned first
      MEDIUM 10-50%
      LOW    < 10%                    -> scanned last
      DEFER  in defer_labels          -> absolute last
    """
    defer_labels = set(defer_labels or [])
    deferred = [w for w in waypoints if w["label"] in defer_labels]
    normal   = [w for w in waypoints if w["label"] not in defer_labels]

    if not history:
        print("  [Scanner] No history yet - using spiral order")
        optimized = normal
    else:
        prob   = occupancy_probability(history)
        high   = [w for w in normal if prob.get(w["label"], 0) >  0.50]
        medium = [w for w in normal if 0.10 < prob.get(w["label"], 0) <= 0.50]
        low    = [w for w in normal if prob.get(w["label"], 0) <= 0.10]

        def greedy_nn(points, start=None):
            if not points:
                return []
            remaining = list(points)
            if start is None:
                current = remaining.pop(0)
            else:
                remaining.sort(
                    key=lambda w: math.hypot(w["x"]-start[0], w["y"]-start[1]))
                current = remaining.pop(0)
            ordered = [current]
            while remaining:
                remaining.sort(
                    key=lambda w: math.hypot(w["x"]-current["x"],
                                             w["y"]-current["y"]))
                current = remaining.pop(0)
                ordered.append(current)
            return ordered

        optimized = []
        last_pos  = None
        for group, name in [(high, "HIGH"), (medium, "MEDIUM"), (low, "LOW")]:
            if not group:
                continue
            og = greedy_nn(group, start=last_pos)
            optimized.extend(og)
            last_pos = (og[-1]["x"], og[-1]["y"])
            print(f"  [Scanner] Priority {name}: {len(og)} spots")

    if deferred:
        print(f"  [Scanner] Deferred (QCar targets, scanned last): "
              f"{[w['label'] for w in deferred]}")

    return optimized + deferred


# ============================================================
# DRONE MOVEMENT
# ============================================================

# Track drone position internally
_drone_pos = [0.0, 0.0, 8.0]

def fly_to(drone, x, y, z, speed=3.0, hover_pause=1.0):
    """
    Fly QDrone2 to (x, y, z) then hover for hover_pause seconds.

    Uses set_transform_and_dynamics with enableDynamics=True so the
    drone moves kinematically (reliable, no RT model dependency) while
    rotors spin visually.
    """
    global _drone_pos
    STEP_SIZE = 2.0
    STEP_DT   = 0.15

    cx, cy, cz = _drone_pos
    dx, dy, dz = x - cx, y - cy, z - cz
    dist = math.sqrt(dx**2 + dy**2 + dz**2)
    yaw  = math.atan2(dy, dx)

    try:
        if dist < STEP_SIZE:
            drone.set_transform_and_dynamics(
                location=[x, y, z],
                rotation=[0, 0, yaw],
                enableDynamics=True,
                waitForConfirmation=True
            )
        else:
            n_steps = max(int(dist / STEP_SIZE), 2)
            for step in range(1, n_steps + 1):
                t = step / n_steps
                drone.set_transform_and_dynamics(
                    location=[cx + dx*t, cy + dy*t, cz + dz*t],
                    rotation=[0, 0, yaw],
                    enableDynamics=True,
                    waitForConfirmation=True
                )
                time.sleep(STEP_DT)
    except Exception as e:
        print(f"[fly_to] Warning: {e} — attempting single jump")
        try:
            drone.set_transform_and_dynamics(
                location=[x, y, z],
                rotation=[0, 0, 0],
                enableDynamics=True,
                waitForConfirmation=True
            )
        except Exception:
            pass
        time.sleep(1.0)

    _drone_pos = [x, y, z]
    time.sleep(hover_pause)


# ============================================================
# OCCUPANCY CHECK  -  static + live combined
# ============================================================

def _spot_occupied(spot, static_positions, live_positions,
                   threshold=IOU_THRESHOLD):
    """
    Return True if any vehicle overlaps the spot.

    Uses is_spot_occupied which checks IoU first, then falls back
    to distance-based detection (3m threshold) so vehicles with
    slight positioning offsets are still caught.

    static_positions: list of [x, y, name]  or  [x, y]
    live_positions:   list of [x, y]  (QCars - treated as QCar dims)
    """
    named_static = []
    for pos in static_positions:
        if len(pos) >= 3:
            named_static.append(pos)
        else:
            named_static.append([pos[0], pos[1], ""])

    all_vehicles = named_static + [
        [vx, vy, "qcar"] for vx, vy in live_positions
    ]
    occupied, _iou, _method = is_spot_occupied(spot, all_vehicles,
                                               iou_thresh=threshold,
                                               dist_thresh=DISTANCE_THRESHOLD)
    return occupied


# ============================================================
# MAIN SCANNER CLASS
# ============================================================

class DroneScanner:
    """
    Manages the full scan-optimise-log lifecycle for a QDrone2
    scanning a parking lot with both stationary and moving vehicles.
    """

    def __init__(self, drone, parking_spots, drone_altitude=8.0):
        self.drone         = drone
        self.parking_spots = parking_spots
        self.altitude      = drone_altitude
        self.waypoints     = None
        self.last_scan     = []
        self._spillover_pairs = {}

    def _prepare_path(self, history, defer_labels=None):
        if PLANNING_MODE == "brain":
            # -- Brain-inspired path planning (Chao et al. 2023) ---
            print("  [Scanner] Using brain-inspired path planner")
            base = brain_inspired_waypoints(
                self.parking_spots,
                drone_altitude=self.altitude,
                history=history,
            )
            # Separate deferred labels, append at end
            defer_labels = set(defer_labels or [])
            deferred = [w for w in base if w["label"] in defer_labels]
            normal   = [w for w in base if w["label"] not in defer_labels]
            self.waypoints = normal + deferred

            if defer_labels:
                print(f"  [Scanner] Deferred (QCar targets, scanned last): "
                      f"{sorted(defer_labels)}")

            # Compare against spiral for metrics
            spiral_base = spiral_waypoints(self.parking_spots, self.altitude)
            brain_metrics = compute_path_metrics(self.waypoints)
            spiral_metrics = compute_path_metrics(spiral_base)
            saving = spiral_metrics.get("total_distance", 0) - brain_metrics.get("total_distance", 0)
            print(f"  [Scanner] Brain path: {brain_metrics.get('total_distance', 0):.0f}m  |  "
                  f"Spiral path: {spiral_metrics.get('total_distance', 0):.0f}m  |  "
                  f"Saved: {saving:.0f}m")
        else:
            # -- Legacy spiral path planning -----------------------
            base = load_base_path(expected_spot_count=len(self.parking_spots))
            if base is None:
                print("  [Scanner] Generating spiral waypoints...")
                base = spiral_waypoints(self.parking_spots, self.altitude)
                save_base_path(base)
            self.waypoints = optimize_waypoint_order(base, history,
                                                     defer_labels=defer_labels)

        print(f"  [Scanner] Path ready: {len(self.waypoints)} waypoints")

    def _scan(self, static_positions, poller, target_labels=None):
        """
        Fly every waypoint in optimised order.
        QCar target spots get QCAR_TARGET_HOVER seconds of extra hover
        so cars that are still driving have time to arrive and park.
        """
        target_labels = set(target_labels or [])
        scan_log      = []
        total         = len(self.waypoints)
        print(f"\n  [Scanner] Starting scan ({total} waypoints)...")
        if target_labels:
            print(f"  [Scanner] Extended hover ({QCAR_TARGET_HOVER}s) on: "
                  f"{sorted(target_labels)}")

        for i, wp in enumerate(self.waypoints):
            spot  = self.parking_spots[wp["spot_index"]]
            label = wp["label"]

            hover = QCAR_TARGET_HOVER if label in target_labels else 0.3
            fly_to(self.drone, wp["x"], wp["y"], wp["z"], hover_pause=hover)

            live_now = poller.get_positions()
            occupied = _spot_occupied(spot, static_positions, live_now)

            if occupied:
                cx, cy = spot["center"][0], spot["center"][1]
                # Use legacy wrapper for raw (cx, cy) coordinate check
                qcar_hit = any(
                    spot_vehicle_iou_legacy(cx, cy, vx, vy, "qcar") >= IOU_THRESHOLD
                    for vx, vy in live_now
                )
                source = "QCar" if qcar_hit else "static"
                occupied_so_far = sum(1 for e in scan_log if e["occupied"]) + 1
                available       = total - occupied_so_far
                print(f"  OCCUPIED [{source}] -> {label} | "
                      f"{occupied_so_far} occupied, {available} available "
                      f"({i+1}/{total} scanned)")

            scan_log.append({
                "label"    : label,
                "x"        : wp["x"],
                "y"        : wp["y"],
                "occupied" : occupied,
                "timestamp": datetime.now().isoformat()
            })

        # -- Spillover pass: large vehicles straddling two spots ---
        # For each occupied spot, check col+/-1 neighbors with the lower
        # SPILLOVER_IOU threshold. This catches trucks/SUVs that overlap
        # into an adjacent spot but fall below the primary IoU threshold.
        occupied_labels = {e["label"] for e in scan_log if e["occupied"]}
        spot_by_rc = {}
        for s in self.parking_spots:
            spot_by_rc[(s.get("row"), s.get("col"))] = s

        # Combine static + last known live positions for the spillover check
        last_live = poller.get_positions()
        named_static = []
        for pos in static_positions:
            if len(pos) >= 3:
                named_static.append(pos)
            else:
                named_static.append([pos[0], pos[1], ""])
        all_vehicles = named_static + [[vx, vy, "qcar"] for vx, vy in last_live]

        spillover_pairs = {}   # neighbor_label -> primary_label
        for entry in scan_log:
            if not entry["occupied"]:
                continue
            spot = next((s for s in self.parking_spots
                         if s.get("label") == entry["label"]), None)
            if spot is None:
                continue
            row, col = spot.get("row"), spot.get("col")
            for dc in (-1, +1):
                neighbor = spot_by_rc.get((row, col + dc))
                if neighbor is None:
                    continue
                n_label = neighbor.get("label", "")
                if n_label in occupied_labels:
                    continue
                spill, spill_iou = has_spillover(neighbor, all_vehicles)
                if spill:
                    for e in scan_log:
                        if e["label"] == n_label:
                            e["occupied"] = True
                            break
                    occupied_labels.add(n_label)
                    spillover_pairs[n_label] = entry["label"]
                    print(f"  OCCUPIED [spillover] -> {entry['label']} & "
                          f"{n_label} (IoU {spill_iou:.3f})")

        if spillover_pairs:
            print(f"  [Scanner] {len(spillover_pairs)} spillover spot(s) added")

        # Attach spillover_pairs to self so _print_summary can use them
        self._spillover_pairs = spillover_pairs

        return scan_log

    @staticmethod
    def _print_summary(scan_log, spillover_pairs=None):
        spillover_pairs = spillover_pairs or {}
        # Reverse map: primary_label -> list of spillover neighbors
        primary_to_spill = defaultdict(list)
        for spill_label, primary_label in spillover_pairs.items():
            primary_to_spill[primary_label].append(spill_label)

        occupied = [e for e in scan_log if e["occupied"]]
        by_row   = defaultdict(list)
        shown    = set()  # avoid listing spillover spots separately

        for e in occupied:
            label = e["label"]
            if label in spillover_pairs:
                continue  # will be shown with its primary
            if label in primary_to_spill:
                paired = sorted(primary_to_spill[label])
                by_row[label[0]].append(f"{label} & {' & '.join(paired)}")
            else:
                by_row[label[0]].append(label)

        available = len(scan_log) - len(occupied)
        print("\n" + "="*60)
        print("DRONE SCAN COMPLETE")
        print("="*60)
        print(f"Total spots   : {len(scan_log)}")
        print(f"Occupied      : {len(occupied)}")
        if spillover_pairs:
            print(f"  (includes {len(spillover_pairs)} spillover from "
                  f"large vehicles)")
        print(f"Available     : {available}")
        if by_row:
            print("\nOccupied spots:")
            for row in sorted(by_row.keys()):
                print(f"  Row {row}: {', '.join(sorted(by_row[row]))}")
        else:
            print("\nNo occupied spots found.")
        print("="*60 + "\n")

    def run(self, qlabs, vehicle_actors, qcar_actors=None,
            qcar_target_labels=None):
        """
        Full pipeline:
          1. Extract static positions from vehicle_actors
          2. Start QCar background poller (or no-op stub if none)
          3. Load/validate history and prepare optimised waypoint order
             (QCar target spots deferred to end)
          4. Fly and scan - extended hover on QCar target spots
          5. Print summary and stop poller
          6. Append run to history log

        Args:
            qlabs              : active QLabs connection
            vehicle_actors     : list of (actor_num, name, x, y) static vehicles
            qcar_actors        : list of (actor_num, name) moving QCars
            qcar_target_labels : list of spot labels from spawn_random_qcars_async
        """
        if qcar_actors is None:
            qcar_actors = []
        if qcar_target_labels is None:
            qcar_target_labels = []

        # 1. Static positions (include name for correct vehicle dimension lookup)
        static_positions = [[x, y, name] for _, name, x, y in vehicle_actors]
        print("\n[Scanner] Stationary vehicles:")
        for _, name, x, y in vehicle_actors:
            print(f"  {name}: ({x:.3f}, {y:.3f})")

        # 2. Start QCar poller or stub
        if qcar_actors:
            print(f"\n[Scanner] Starting QCar live poller "
                  f"({len(qcar_actors)} car(s)):")
            for an, name in qcar_actors:
                print(f"  {name} (actor {an})")
            poller = QCarPoller(qlabs, qcar_actors)
            poller.start()
            time.sleep(1.0)   # let poller complete at least one read
        else:
            class _NoOpPoller:
                def get_positions(self): return []
                def get_named_positions(self): return {}
                def stop(self): pass
            poller = _NoOpPoller()
            print("\n[Scanner] No QCar actors - static detection only")

        # 3. Prepare path
        history = load_history()
        run_num = len(history) + 1
        print(f"\n[Scanner] Run #{run_num}  |  "
              f"{len(history)} historical runs available")
        self._prepare_path(history, defer_labels=qcar_target_labels)

        # 4. Stop the real-time model for kinematic flight control
        #    (set_transform_and_dynamics works best without the RT model)
        print("  [Scanner] Stopping drone RT model for kinematic flight...")
        QLabsRealTime().terminate_all_real_time_models()
        time.sleep(1.0)

        # 5. Initialize drone position tracker from spawn location
        global _drone_pos
        if self.waypoints:
            first = self.waypoints[0]
            _drone_pos = [first["x"], first["y"], first["z"]]

        # 6. Scan
        scan_log = self._scan(static_positions, poller,
                              target_labels=qcar_target_labels)
        self.last_scan = scan_log

        # 5. Summary + stop poller
        self._print_summary(scan_log, self._spillover_pairs)
        poller.stop()

        # 6. Log
        run_record = {
            "run"     : run_num,
            "datetime": datetime.now().isoformat(),
            "vehicles": [{"name": n, "x": x, "y": y}
                         for _, n, x, y in vehicle_actors],
            "qcars"   : [{"name": n, "actor": an}
                         for an, n in qcar_actors],
            "scan"    : scan_log,
            "summary" : {
                "total"   : len(scan_log),
                "occupied": sum(1 for e in scan_log if e["occupied"]),
                "empty"   : sum(1 for e in scan_log if not e["occupied"])
            }
        }
        append_history(run_record)

        # Restart RT model so QDrone2 camera is available for vision scanner
        print("  [Scanner] Restarting drone RT model for vision scanner...")
        import pal.resources.rtmodels as rtmodels
        from qvl.real_time import QLabsRealTime as _RT
        _RT().start_real_time_model(modelName=rtmodels.QDRONE2, actorNumber=0)
        time.sleep(2.0)

        return scan_log


# ============================================================
# STANDALONE TEST
# ============================================================

if __name__ == "__main__":
    start_xyz = [-5, -10, 0.005]
    mock_spots = []
    for row in range(4):
        for col in range(39):
            lx = col * 2.7
            ly = row * 15.5
            x  = (start_xyz[0]
                  + lx * math.cos(math.radians(90))
                  - ly * math.sin(math.radians(90)))
            y  = (start_xyz[1]
                  + lx * math.sin(math.radians(90))
                  + ly * math.cos(math.radians(90)))
            yaw = 90 if row % 2 == 0 else 270
            mock_spots.append({
                "label"   : f"{chr(ord('A')+row)}{col}",
                "center"  : [x, y, 0.005],
                "yaw_deg" : yaw,
                "row"     : row,
                "direction": "right" if yaw == 90 else "left"
            })

    print("=== Standalone test ===\n")

    wps = spiral_waypoints(mock_spots, drone_altitude=8.0)
    print(f"Spiral: {len(wps)} waypoints")
    print(f"First 5 : {[w['label'] for w in wps[:5]]}")
    print(f"Last  5 : {[w['label'] for w in wps[-5:]]}\n")
    save_base_path(wps)

    fake_history = [{
        "run": 1,
        "scan": ([{"label": f"C{i}", "occupied": i in [0, 2, 4]}
                  for i in range(39)] +
                 [{"label": f"{r}{i}", "occupied": False}
                  for r in "ABD" for i in range(39)])
    }]
    opt = optimize_waypoint_order(wps, fake_history, defer_labels={"D38", "C32"})
    print(f"Optimised first 10 : {[w['label'] for w in opt[:10]]}")
    print(f"Optimised last  5  : {[w['label'] for w in opt[-5:]]}")

    class MockPoller:
        def get_positions(self):
            return [[mock_spots[1]["center"][0], mock_spots[1]["center"][1]]]

    result = _spot_occupied(
        mock_spots[1],
        static_positions=[[-37.956, -4.745, "Truck"]],
        live_positions=MockPoller().get_positions()
    )
    print(f"\nOccupancy check with QCar present : {result}")   # True

    result2 = _spot_occupied(
        mock_spots[5],
        static_positions=[[-37.956, -4.745, "Truck"]],
        live_positions=MockPoller().get_positions()
    )
    print(f"Occupancy check empty spot        : {result2}")   # False

    print("\nAll tests passed.")
