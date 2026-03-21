"""
multi_drone_scanner.py
----------------------
Parallel zone-based scanning using all 3 QDrone2 actors.

Strategy:
    - Split 156 spots into 3 zones (52 spots each)
    - Each drone covers one zone simultaneously in a thread
    - Total scan time reduced ~3x vs single drone

Zone assignment (based on lot layout):
    Drone 0 [-40, 100, 25]  -> Section 1 rows A+B  (cols 0-39)
    Drone 1 [10,  -20, 25]  -> Section 1 rows C+D  (cols 0-39)
    Drone 2 [-90, -20, 25]  -> Section 2 all rows  (cols 0-39)
"""

import threading
import time
import math
from datetime import datetime
from collections import defaultdict

from occupancy import (spot_vehicle_iou_legacy, is_spot_occupied,
                       IOU_THRESHOLD, DISTANCE_THRESHOLD)


# ── Per-drone position tracker (one per drone actor) ─────────────────────────
_drone_positions = {}   # actor_number -> [x, y, z]

STEP_SIZE = 4.0    # metres per interpolation step
STEP_DT   = 0.15   # seconds between steps per drone
HOVER_TIME = 0.3   # seconds hover per waypoint


def _fly_to_drone(drone, actor_num, x, y, z, hover_pause=HOVER_TIME):
    """
    Fly a specific drone to (x, y, z) using kinematic control.

    Uses enableDynamics=False — reliable kinematic movement.
    No global lock — each drone manages its own QLabs connection
    independently. waitForConfirmation=False used to avoid blocking
    other drone threads while waiting for ACK.
    """
    global _drone_positions

    cx, cy, cz = _drone_positions.get(actor_num, [x, y, z])
    dx, dy, dz = x - cx, y - cy, z - cz
    dist = math.sqrt(dx**2 + dy**2 + dz**2)
    yaw  = math.atan2(dy, dx)

    try:
        if dist < STEP_SIZE:
            drone.set_transform_and_dynamics(
                location=[x, y, z],
                rotation=[0, 0, yaw],
                enableDynamics=False,
                waitForConfirmation=False
            )
            time.sleep(STEP_DT)
        else:
            n_steps = max(int(dist / STEP_SIZE), 2)
            for step in range(1, n_steps + 1):
                t = step / n_steps
                drone.set_transform_and_dynamics(
                    location=[cx + dx*t, cy + dy*t, cz + dz*t],
                    rotation=[0, 0, yaw],
                    enableDynamics=False,
                    waitForConfirmation=False
                )
                time.sleep(STEP_DT)
    except Exception as e:
        print(f"  [Drone {actor_num}] fly_to warning: {e}")
        try:
            drone.set_transform_and_dynamics(
                location=[x, y, z],
                rotation=[0, 0, 0],
                enableDynamics=False,
                waitForConfirmation=False
            )
        except Exception:
            pass
        time.sleep(0.5)

    _drone_positions[actor_num] = [x, y, z]
    time.sleep(hover_pause)


# ── Zone assignment ───────────────────────────────────────────────────────────

def assign_zones(parking_spots, n_drones=3):
    """
    Split parking spots into n_drones equal zones.

    Sorts spots by section then row then column so each drone covers
    a geographically contiguous area, minimising travel distance.
    Splits are balanced — each drone gets floor(total/n) or ceil(total/n) spots.
    """
    # Sort for contiguous zones
    sorted_spots = sorted(
        parking_spots,
        key=lambda s: (
            s.get("section", 1),
            s.get("row", 0),
            s.get("col", 0)
        )
    )

    total  = len(sorted_spots)
    base   = total // n_drones
    extras = total % n_drones   # first `extras` drones get one more spot

    zones = []
    idx = 0
    for i in range(n_drones):
        size = base + (1 if i < extras else 0)
        zones.append(sorted_spots[idx: idx + size])
        idx += size

    for i, zone in enumerate(zones):
        print(f"  [MultiDrone] Drone {i} zone: {len(zone)} spots "
              f"({zone[0]['label'] if zone else 'none'} -> "
              f"{zone[-1]['label'] if zone else 'none'})")

    return zones


# ── Zone scanner (runs in its own thread) ─────────────────────────────────────

class ZoneScanner(threading.Thread):
    """
    Scans a subset of parking spots with a single drone.
    Results are written to shared scan_log list (thread-safe via lock).
    """

    def __init__(self, drone, actor_num, spawn_pos, spots, altitude,
                 static_positions, poller, scan_log, log_lock,
                 target_labels=None, qcar_hover=3.0):
        super().__init__(daemon=True)
        self.drone            = drone
        self.actor_num        = actor_num
        self.spots            = spots
        self.altitude         = altitude
        self.static_positions = static_positions
        self.poller           = poller
        self.scan_log         = scan_log
        self.log_lock         = log_lock
        self.target_labels    = set(target_labels or [])
        self.qcar_hover       = qcar_hover
        self.done             = False

        # Initialise position tracker for this drone
        _drone_positions[actor_num] = list(spawn_pos)

    def run(self):
        print(f"  [Drone {self.actor_num}] Starting zone scan "
              f"({len(self.spots)} spots)")

        for spot in self.spots:
            label = spot.get("label", "?")
            cx = spot["center"][0]
            cy = spot["center"][1]
            cz = spot["center"][2] + self.altitude

            hover = self.qcar_hover if label in self.target_labels else HOVER_TIME
            _fly_to_drone(self.drone, self.actor_num, cx, cy, cz,
                          hover_pause=hover)

            # Occupancy check
            live_now = self.poller.get_positions()
            all_vehicles = self.static_positions + [
                [vx, vy, "qcar"] for vx, vy in live_now
            ]
            occupied, _iou, _method = is_spot_occupied(
                spot, all_vehicles,
                iou_thresh=IOU_THRESHOLD,
                dist_thresh=DISTANCE_THRESHOLD
            )

            entry = {
                "label"    : label,
                "x"        : cx,
                "y"        : cy,
                "occupied" : occupied,
                "drone"    : self.actor_num,
                "timestamp": datetime.now().isoformat()
            }

            with self.log_lock:
                self.scan_log.append(entry)
                if occupied:
                    print(f"  OCCUPIED [drone {self.actor_num}] -> {label}")

        self.done = True
        print(f"  [Drone {self.actor_num}] Zone scan complete")


# ── Public API ────────────────────────────────────────────────────────────────

def parallel_scan(drones, spawn_positions, parking_spots, altitude,
                  vehicle_actors, poller,
                  target_labels=None, qcar_hover=3.0):
    """
    Scan the full lot using all available drones in parallel.

    Parameters
    ----------
    drones          : list of QLabsQDrone2 objects (one per drone)
    spawn_positions : list of [x, y, z] — actual spawn coords per drone
    parking_spots   : full list of spot dicts from build_parking_lot()
    altitude        : scan altitude in metres above spot z
    vehicle_actors  : list of (actor_num, name, x, y) static vehicles
    poller          : QCarPoller instance (or NoOpPoller)
    target_labels   : spot labels that need extended hover (QCar targets)
    qcar_hover      : hover time in seconds over QCar target spots

    Returns
    -------
    scan_log : list of scan entry dicts, same format as single-drone scan
    """
    n_drones = len(drones)
    zones    = assign_zones(parking_spots, n_drones)

    static_positions = [[x, y, name] for _, name, x, y in vehicle_actors]

    # Shared results
    scan_log = []
    log_lock = threading.Lock()

    # Launch one thread per drone
    scanners = []
    for i, (drone, spawn_pos, zone) in enumerate(
            zip(drones, spawn_positions, zones)):
        if not zone:
            continue
        s = ZoneScanner(
            drone=drone,
            actor_num=i,
            spawn_pos=spawn_pos,
            spots=zone,
            altitude=altitude,
            static_positions=static_positions,
            poller=poller,
            scan_log=scan_log,
            log_lock=log_lock,
            target_labels=target_labels,
            qcar_hover=qcar_hover
        )
        scanners.append(s)

    start = time.time()
    print(f"\n[MultiDrone] Starting parallel scan with {len(scanners)} drones...")

    # Stagger drone launches by 1 second each to prevent all drones
    # sending commands simultaneously on the first move
    for i, s in enumerate(scanners):
        s.start()
        if i < len(scanners) - 1:
            time.sleep(1.0)

    # Wait for all drones to finish
    for s in scanners:
        s.join()

    elapsed = time.time() - start
    occupied = sum(1 for e in scan_log if e["occupied"])
    print(f"\n[MultiDrone] Scan complete in {elapsed:.1f}s")
    print(f"  Total: {len(scan_log)} | Occupied: {occupied} | "
          f"Available: {len(scan_log) - occupied}")

    return scan_log


def print_parallel_summary(scan_log):
    """Print formatted scan results."""
    occupied = [e for e in scan_log if e["occupied"]]
    by_row   = defaultdict(list)
    for e in occupied:
        by_row[e["label"][0]].append(e["label"])

    print("\n" + "="*60)
    print("PARALLEL DRONE SCAN COMPLETE")
    print("="*60)
    print(f"Total spots   : {len(scan_log)}")
    print(f"Occupied      : {len(occupied)}")
    print(f"Available     : {len(scan_log) - len(occupied)}")
    if by_row:
        print("\nOccupied spots:")
        for row in sorted(by_row.keys()):
            print(f"  Row {row}: {', '.join(sorted(by_row[row]))}")
    print("="*60 + "\n")
