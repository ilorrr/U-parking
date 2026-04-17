# ============================================================
# live_scanner.py
# Velocity-based drone scanner with continuous camera access.
#
# Uses set_velocity_and_request_state instead of
# set_transform_and_dynamics, so the RT model stays alive
# and get_image works throughout the entire scan.
#
# Each velocity call returns the drone's current position,
# so the pipe stays clean (no orphaned ACKs).
# ============================================================

import os
import sys
import math
import time
import json
import cv2
import numpy as np
from datetime import datetime
from collections import defaultdict

QLABS_LIB_PATH = r"C:\Users\Roli\Documents\uparking"
if QLABS_LIB_PATH not in sys.path:
    sys.path.insert(0, QLABS_LIB_PATH)

# Camera constant
CAMERA_DOWNWARD = 5

# Flight parameters
MAX_SPEED       = 5.0     # m/s horizontal
MAX_VERT_SPEED  = 3.0     # m/s vertical
ARRIVAL_THRESH  = 1.5     # metres — close enough to capture
HOVER_TIME      = 0.5     # seconds to hover before capture
CONTROL_DT      = 0.05    # seconds between velocity commands
KP_HORIZ        = 1.5     # proportional gain horizontal
KP_VERT         = 2.0     # proportional gain vertical


class LiveScanner:
    """
    Flies a QDrone2 using velocity commands and captures frames
    from the downward camera at each parking spot.

    The key difference from vision_collection_pass:
      - Uses set_velocity_and_request_state (request-response)
        instead of set_transform_and_dynamics (fire-and-forget)
      - Every velocity call reads its own response → pipe stays clean
      - RT model stays alive → get_image works the entire scan
      - Drone physically flies → looks realistic for demos

    Usage:
        scanner = LiveScanner(drone, parking_spots)
        results = scanner.scan(scan_log, on_frame=my_callback)
    """

    def __init__(self, drone, parking_spots, altitude=8.0,
                 save_dir="vision_data"):
        self.drone         = drone
        self.parking_spots = parking_spots
        self.altitude      = altitude
        self.save_dir      = save_dir
        self._current_pos  = None   # updated by velocity calls

    def _fly_toward(self, target_x, target_y, target_z):
        """
        Send one velocity command toward the target position.
        Returns (at_target, current_position).

        Uses set_velocity_and_request_state which:
          1. Sends velocity command
          2. Reads response (position, orientation, etc.)
          3. Pipe is clean — no stale ACKs
        """
        # Get current state from velocity command response
        # First call with zero velocity just to get position
        if self._current_pos is None:
            ok, loc, ori, *_ = self.drone.set_velocity_and_request_state_degrees(
                motorsEnabled=True,
                velocity=[0, 0, 0],
                orientation=[0, 0, 0]
            )
            if ok:
                self._current_pos = loc
            else:
                return False, [0, 0, 0]

        cx, cy, cz = self._current_pos

        # Compute error
        dx = target_x - cx
        dy = target_y - cy
        dz = target_z - cz
        horiz_dist = math.sqrt(dx**2 + dy**2)
        total_dist = math.sqrt(dx**2 + dy**2 + dz**2)

        # Check arrival
        if total_dist < ARRIVAL_THRESH:
            # Hover — zero velocity
            ok, loc, ori, *_ = self.drone.set_velocity_and_request_state_degrees(
                motorsEnabled=True,
                velocity=[0, 0, 0],
                orientation=[0, 0, 0]
            )
            if ok:
                self._current_pos = loc
            return True, self._current_pos

        # Compute yaw to face target
        yaw_deg = math.degrees(math.atan2(dy, dx))

        # Proportional velocity (body frame: x=forward, z=up)
        fwd_speed = min(MAX_SPEED, horiz_dist * KP_HORIZ)
        vert_speed = max(-MAX_VERT_SPEED,
                         min(MAX_VERT_SPEED, dz * KP_VERT))

        # Send velocity command — THIS is why the pipe stays clean
        # Each call sends a command AND reads the full response
        ok, loc, ori, *_ = self.drone.set_velocity_and_request_state_degrees(
            motorsEnabled=True,
            velocity=[fwd_speed, 0, vert_speed],
            orientation=[0, 0, yaw_deg]
        )

        if ok:
            self._current_pos = loc

        return False, self._current_pos

    def _fly_to_waypoint(self, x, y, z, timeout=30.0):
        """
        Fly to (x, y, z) using proportional velocity control.
        Returns True if arrived, False if timed out.
        """
        start = time.time()
        while time.time() - start < timeout:
            arrived, pos = self._fly_toward(x, y, z)
            if arrived:
                return True
            time.sleep(CONTROL_DT)
        return False

    def _capture_frame(self):
        """
        Capture a frame from the drone's downward camera.
        Returns the frame or None.
        """
        try:
            success, cam_num, frame = self.drone.get_image(CAMERA_DOWNWARD)
            if success and frame is not None:
                return frame
        except Exception as e:
            print(f"  [LiveScan] Capture error: {e}")
        return None

    def scan(self, scan_log, sample_empty=0.3, on_frame=None):
        """
        Fly drone 0 to each spot, capture frames, classify.

        Parameters
        ----------
        scan_log      : occupancy labels from parallel scan or IoU pre-scan
        sample_empty  : fraction of empty spots to visit (0.0-1.0)
        on_frame      : optional callback(label, frame, occupied) called
                        after each capture — use for WebSocket streaming

        Returns
        -------
        dict with capture stats
        """
        import random

        print("\n" + "="*60)
        print("LIVE DRONE SCAN + CAPTURE")
        print("="*60)

        # Build lookups
        label_status  = {e["label"]: e["occupied"] for e in scan_log}
        spot_by_label = {s.get("label"): s for s in self.parking_spots}

        # Build visit list
        occupied_labels = [l for l, occ in label_status.items() if occ]
        empty_labels    = [l for l, occ in label_status.items() if not occ]
        n_empty = max(1, int(len(empty_labels) * sample_empty))
        sampled_empty = random.sample(empty_labels,
                                       min(n_empty, len(empty_labels)))

        visit_list = [(l, True) for l in occupied_labels] + \
                     [(l, False) for l in sampled_empty]

        # Sort spatially — row-by-row lawnmower
        def _sort_key(item):
            label, _ = item
            spot = spot_by_label.get(label)
            if not spot:
                return (99, 99, 99)
            section = spot.get("section", 1)
            row     = spot.get("row", 0)
            col     = spot.get("col", 0)
            col_key = col if row % 2 == 0 else -col
            return (section, row, col_key)

        visit_list.sort(key=_sort_key)

        print(f"  Occupied: {len(occupied_labels)}")
        print(f"  Empty:    {len(sampled_empty)} "
              f"({sample_empty:.0%} of {len(empty_labels)})")
        print(f"  Total:    {len(visit_list)} waypoints")
        print(f"  Mode:     velocity flight + live camera")

        # Create output dirs
        occ_dir   = os.path.join(self.save_dir, "occupied")
        empty_dir = os.path.join(self.save_dir, "empty")
        os.makedirs(occ_dir, exist_ok=True)
        os.makedirs(empty_dir, exist_ok=True)

        occ_count   = len([f for f in os.listdir(occ_dir)
                           if f.endswith(".jpg")])
        empty_count = len([f for f in os.listdir(empty_dir)
                           if f.endswith(".jpg")])

        captured = 0
        failed   = 0
        start    = time.time()

        # Initialize drone motors
        print("  [LiveScan] Enabling motors...")
        self.drone.set_velocity_and_request_state_degrees(
            motorsEnabled=True, velocity=[0, 0, 0], orientation=[0, 0, 0])
        time.sleep(1.0)

        for i, (label, occupied) in enumerate(visit_list):
            spot = spot_by_label.get(label)
            if not spot:
                continue

            cx = spot["center"][0]
            cy = spot["center"][1]
            cz = spot["center"][2] + self.altitude

            # Fly to spot
            arrived = self._fly_to_waypoint(cx, cy, cz)
            if not arrived:
                print(f"  [LiveScan] Timeout reaching {label}")
                failed += 1
                continue

            # Hover to stabilize
            time.sleep(HOVER_TIME)

            # Capture frame — camera works because pipe is clean
            frame = self._capture_frame()
            if frame is None:
                failed += 1
                if failed <= 5:
                    print(f"  [LiveScan] No frame at {label} "
                          f"({failed} failures)")
                continue

            # Callback for live streaming
            if on_frame:
                try:
                    on_frame(label, frame, occupied)
                except Exception as e:
                    print(f"  [LiveScan] Stream callback error: {e}")

            # Save frame
            if occupied:
                filename = f"{label}_{occ_count:05d}.jpg"
                filepath = os.path.join(occ_dir, filename)
                occ_count += 1
            else:
                filename = f"{label}_{empty_count:05d}.jpg"
                filepath = os.path.join(empty_dir, filename)
                empty_count += 1

            cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            captured += 1

            if (i + 1) % 20 == 0:
                elapsed_so_far = time.time() - start
                print(f"  [LiveScan] {i+1}/{len(visit_list)} | "
                      f"{captured} captured | {elapsed_so_far:.0f}s")

        # Stop motors
        self.drone.set_velocity_and_request_state_degrees(
            motorsEnabled=False, velocity=[0, 0, 0], orientation=[0, 0, 0])

        elapsed = time.time() - start

        print(f"\n  [LiveScan] Complete in {elapsed:.1f}s")
        print(f"    Captured: {captured}")
        print(f"    Failed:   {failed}")
        print(f"    Occupied: {occ_count} total in {occ_dir}")
        print(f"    Empty:    {empty_count} total in {empty_dir}")
        print("="*60 + "\n")

        return {"occupied": occ_count, "empty": empty_count,
                "captured": captured, "failed": failed}


# ============================================================
# WEBSOCKET FRAME STREAMER (optional callback for live scan)
# ============================================================

def make_ws_streamer(ws_server=None):
    """
    Returns a callback function for LiveScanner.scan(on_frame=...).

    Each captured frame gets base64-encoded and broadcast
    to all connected WebSocket clients.

    Usage:
        streamer = make_ws_streamer(ws_server)
        scanner.scan(scan_log, on_frame=streamer)
    """
    import base64

    def stream_frame(label, frame, occupied):
        if ws_server is None:
            return

        # Encode frame as JPEG → base64 for WebSocket transport
        _, buffer = cv2.imencode('.jpg', frame,
                                  [cv2.IMWRITE_JPEG_QUALITY, 70])
        b64 = base64.b64encode(buffer).decode('utf-8')

        msg = json.dumps({
            "type"     : "drone_frame",
            "spot"     : label,
            "occupied" : occupied,
            "frame"    : b64,
            "timestamp": datetime.now().isoformat()
        })

        try:
            ws_server.broadcast(msg)
        except Exception:
            pass   # don't crash scan if WS fails

    return stream_frame


# ============================================================
# BACKEND PUSH (optional — POST results to Django)
# ============================================================

def push_to_backend(scan_log, backend_url="http://localhost:8000/api/occupancy-update/"):
    """
    Push scan results to Django backend for dashboard updates.
    Groups spots by section and sends one POST per section.
    """
    try:
        import requests
    except ImportError:
        print("  [Backend] requests not installed — skipping push")
        return

    for section in [1, 2]:
        spots = {
            e["label"]: e["occupied"]
            for e in scan_log
            if e["label"].startswith(f"S{section}-")
        }
        if not spots:
            continue

        try:
            response = requests.post(backend_url, json={
                "section": section,
                "spots": spots
            }, timeout=5)
            occ = sum(1 for v in spots.values() if v)
            print(f"  [Backend] S{section}: {occ}/{len(spots)} "
                  f"occupied → {response.status_code}")
        except Exception as e:
            print(f"  [Backend] S{section} push failed: {e}")
