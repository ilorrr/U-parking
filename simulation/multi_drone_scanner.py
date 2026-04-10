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


# Per-drone position tracker (one per drone actor) 
_drone_positions = {}   # actor_number -> [x, y, z]

STEP_SIZE = 2.0  # metres per interpolation step
STEP_DT   = 1.5   # seconds between steps per drone
HOVER_TIME = 0.2   # seconds hover per waypoint


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


#Zone scanner

class ZoneScanner(threading.Thread):
    """
    Scans a subset of parking spots with a single drone.
    Results are written to shared scan_log list (thread-safe via lock).
    """

    def __init__(self, drone, actor_num, spawn_pos, spots, altitude,
                 static_positions, poller, scan_log, log_lock,
                 target_labels=None, qcar_hover=3.0,update_callback = None):
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
        self.update_callback  = update_callback
        self.done             = False

        # Initialise position tracker for this drone
        _drone_positions[actor_num] = list(spawn_pos)

    def run(self):
        print(f"  [Drone {self.actor_num}] Starting zone scan "
              f"({len(self.spots)} spots)")

        CLUSTER_STEP_SIZE = 3  # Group spots to scan 3 at a time (Grid Consolidation)

        for i in range(0, len(self.spots), CLUSTER_STEP_SIZE):
            cluster_spots = self.spots[i : i + CLUSTER_STEP_SIZE]
            
            # Center the drone on the middle spot of the cluster to capture all
            center_spot = cluster_spots[len(cluster_spots) // 2] 
            
            cx = center_spot["center"][0]
            cy = center_spot["center"][1]
            cz = center_spot["center"][2] + self.altitude

            # Hover longer if the center spot is a QCar target
            hover = self.qcar_hover if center_spot.get("label") in self.target_labels else HOVER_TIME
            _fly_to_drone(self.drone, self.actor_num, cx, cy, cz, hover_pause=hover)

            # Occupancy check for ALL spots in this cluster simultaneously
            live_now = self.poller.get_positions()
            all_vehicles = self.static_positions + [
                [vx, vy, "qcar"] for vx, vy in live_now
            ]
            
            for spot in cluster_spots:
                label = spot.get("label", "?")
                occupied, _iou, _method = is_spot_occupied(
                    spot, all_vehicles,
                    iou_thresh=IOU_THRESHOLD,
                    dist_thresh=DISTANCE_THRESHOLD
                )

                entry = {
                    "label"    : label,
                    "x"        : spot["center"][0],
                    "y"        : spot["center"][1],
                    "occupied" : occupied,
                    "drone"    : self.actor_num,
                    "timestamp": datetime.now().isoformat()
                }

                with self.log_lock:
                    self.scan_log.append(entry)
                    if occupied:
                        print(f"  OCCUPIED [drone {self.actor_num}] -> {label}")
                    
                    if self.update_callback:
                        self.update_callback([entry])

        self.done = True
        print(f"  [Drone {self.actor_num}] Zone scan complete")


# Public API 

def parallel_scan(drones, spawn_positions, parking_spots, altitude,
                  vehicle_actors, poller,
                  target_labels=None, qcar_hover=3.0, update_callback = None):
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
            qcar_hover=qcar_hover,
            update_callback = update_callback
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


# Park idle drones  

def park_idle_drones(drones, active_index=0, park_altitude=50.0,
                     spawn_positions=None):
    """
    Move all drones EXCEPT the active one to a high parking altitude
    and wait for confirmation. This clears the QLabs communication
    channel so only the active drone is sending/receiving.

    Must be called AFTER parallel_scan threads have joined and
    BEFORE vision_collection_pass starts.

    Parameters
    ----------
    drones          : list of QLabsQDrone2 objects
    active_index    : which drone will be used for vision (default 0)
    park_altitude   : how high to park idle drones (out of camera view)
    spawn_positions : original spawn [x,y,z] per drone (park near spawn)
    """
    print(f"\n[MultiDrone] Parking idle drones (keeping drone {active_index})...")

    for i, drone in enumerate(drones):
        if i == active_index:
            continue

        # Park at original spawn X/Y but high up, out of the way
        if spawn_positions and i < len(spawn_positions):
            px, py = spawn_positions[i][0], spawn_positions[i][1]
        else:
            px, py = 200.0 + i * 50, 200.0   # far corner fallback

        try:
            drone.set_transform_and_dynamics(
                location=[px, py, park_altitude],
                rotation=[0, 0, 0],
                enableDynamics=False,
                waitForConfirmation=False
            )
            time.sleep(0.5)
            print(f"  Drone {i} parked at ({px:.0f}, {py:.0f}, {park_altitude:.0f})")
        except Exception as e:
            print(f"  Drone {i} park failed: {e}")

    # Let QLabs settle before drone 0 starts solo work
    time.sleep(2.0)
    print(f"  Drone {active_index} is now the only active drone")


# Vision collection pass (runs after parallel scan)

CAMERA_DOWNWARD = 5

def vision_collection_pass(drone, parking_spots, scan_log, altitude=8.0,
                           save_dir="vision_data", sample_empty=0.3,
                           use_free_camera=False, update_callback=None):
    """
    Single-drone sequential pass to capture camera frames for training.
    Pushes valid frames to the live_streamer queue instead of cv2.imshow.
    """
    import os
    import cv2
    import random

    print("\n" + "="*60)
    print("VISION DATA COLLECTION PASS")
    print("="*60)

    # Build label -> occupied map from scan results
    label_status = {e["label"]: e["occupied"] for e in scan_log}

    # Build label -> spot lookup
    spot_by_label = {s.get("label"): s for s in parking_spots}

    # Separate occupied vs empty
    occupied_labels = [l for l, occ in label_status.items() if occ]
    empty_labels    = [l for l, occ in label_status.items() if not occ]

    # Sample a fraction of empty spots
    n_empty = max(1, int(len(empty_labels) * sample_empty))
    sampled_empty = random.sample(empty_labels, min(n_empty, len(empty_labels)))

    visit_list = [(l, True) for l in occupied_labels] + \
                 [(l, False) for l in sampled_empty]

    def _spot_sort_key(item):
        label, _ = item
        spot = spot_by_label.get(label)
        if spot is None:
            return (99, 99, 99)
        section = spot.get("section", 1)
        row     = spot.get("row", 0)
        col     = spot.get("col", 0)
        col_key = col if row % 2 == 0 else -col
        return (section, row, col_key)

    visit_list.sort(key=_spot_sort_key)
    
    print(f"  Occupied spots to capture: {len(occupied_labels)}")
    print(f"  Empty spots to capture:    {len(sampled_empty)} "
          f"({sample_empty:.0%} of {len(empty_labels)})")
    print(f"  Total waypoints:           {len(visit_list)}")

    # Create output dirs
    occ_dir   = os.path.join(save_dir, "occupied")
    empty_dir = os.path.join(save_dir, "empty")
    os.makedirs(occ_dir, exist_ok=True)
    os.makedirs(empty_dir, exist_ok=True)

    # Count existing files to avoid overwriting
    occ_count   = len([f for f in os.listdir(occ_dir) if f.endswith(".jpg")])
    empty_count = len([f for f in os.listdir(empty_dir) if f.endswith(".jpg")])

    captured  = 0
    failed    = 0
    start     = time.time()
    render_wait = 1.2

    for i, (label, occupied) in enumerate(visit_list):
        spot = spot_by_label.get(label)
        if spot is None:
            continue

        cx = spot["center"][0]
        cy = spot["center"][1]
        cz = spot["center"][2] + altitude

        # Move camera to spot
        try:
            if use_free_camera:
                drone.set_transform_degrees(
                    location=[cx, cy, cz],
                    rotation=[0, 90, 0]
                )
            else:
                drone.set_transform_and_dynamics(
                    location=[cx, cy, cz],
                    rotation=[0, 0, 0],
                    enableDynamics=False,
                    waitForConfirmation=True
                )
        except Exception as e:
            failed += 1
            if failed <= 5:
                print(f"  [Vision] Move failed at {label}: {e}")
            continue

        time.sleep(render_wait)

        # Capture frame with adaptive retry
        frame = None
        try:
            if use_free_camera:
                success, frame = drone.get_image()
            else:
                success, cam_num, frame = drone.get_image(CAMERA_DOWNWARD)

            if not success or frame is None:
                render_wait = min(render_wait + 1.0, 5.0)
                time.sleep(render_wait)
                if use_free_camera:
                    success, frame = drone.get_image()
                else:
                    success, cam_num, frame = drone.get_image(CAMERA_DOWNWARD)

            if not success or frame is None:
                failed += 1
                if failed <= 5:
                    print(f"  [Vision] No frame at {label} ({failed} failures)")
                continue

        except Exception as e:
            failed += 1
            render_wait = min(render_wait + 1.0, 5.0)
            continue
        
        # setting crop 
        # Crop a 250x250 square from the absolute center of the frame
        h, w = frame.shape[:2]
        crop_width = 120
        crop_height = 280
        
        x1 = max(0, (w // 2) - (crop_width // 2))
        y1 = max(0, (h // 2) - (crop_height // 2))
        cropped_patch = frame[y1:y1+crop_height, x1:x1+crop_width]
        
        # Save Image locally
        if occupied:
            filename = f"{label}_{occ_count:05d}.jpg"
            filepath = os.path.join(occ_dir, filename)
            occ_count += 1
        else:
            filename = f"{label}_{empty_count:05d}.jpg"
            filepath = os.path.join(empty_dir, filename)
            empty_count += 1

        cv2.imwrite(filepath,cropped_patch, [cv2.IMWRITE_JPEG_QUALITY, 95])
        captured += 1
        
        if update_callback:
            update_callback([{"label": label, "occupied": occupied}])

        # Live display annotation
        display = frame.copy()
        status_text = "OCCUPIED" if occupied else "EMPTY"
        color = (0, 0, 255) if occupied else (0, 255, 0)
        cv2.putText(display, f"{label} - {status_text}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(display, f"{captured}/{len(visit_list)}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)

        # --- PUSH TO THREAD-SAFE QUEUE INSTEAD OF CV2.IMSHOW ---
        try:
            from live_streamer import frame_queue
            if frame_queue.full():
                frame_queue.get_nowait()
            frame_queue.put_nowait(display)
        except ImportError:
            pass
        # -------------------------------------------------------

        if (i + 1) % 10 == 0 or i == 0:
            elapsed_so_far = time.time() - start
            rate = captured / max(elapsed_so_far, 1)
            remaining = (len(visit_list) - i - 1) / max(rate, 0.01)
            print(f"  [Vision] {i+1}/{len(visit_list)} | "
                  f"{captured} captured, {failed} failed | "
                  f"wait={render_wait:.1f}s | "
                  f"ETA: {remaining/60:.1f}min", flush=True)

    elapsed = time.time() - start

    print(f"\n  [Vision] Collection complete in {elapsed:.1f}s")
    print(f"    Captured: {captured}")
    print(f"    Failed:   {failed}")
    print(f"    Occupied: {occ_count} total in {occ_dir}")
    print(f"    Empty:    {empty_count} total in {empty_dir}")
    print("="*60 + "\n")

    return {"occupied": occ_count, "empty": empty_count,
            "captured": captured, "failed": failed}