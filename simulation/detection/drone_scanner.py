"""
drone_scanner.py
----------------
QDrone2 camera scanning pipeline for uParking.

Integrates:
    - Paper 2 (Peraza-Garzón et al., 2026) detection + occupancy pipeline
    - Paper 4 (Yan et al., 2024) autonomous waypoint patrol loop concept
    - Existing uParking QLabs connection, WebSocket server, and A*/SNN
      path planning modules

Flow at each waypoint:
    QDrone2 camera frame
        → preprocessing.py  (blur check, letterbox, normalize)
        → detector.py       (YOLOv11 dual-class inference)
        → occupancy.py      (IoU matching, Eq.7 percentage)
        → WebSocket         (React dashboard broadcast)
"""

from __future__ import annotations

import asyncio
import cv2
import json
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from .preprocessing import preprocess_frame
from .detector import ParkingDetector, MockParkingDetector, DetectionResult
from .occupancy import (
    match_occupancy,
    build_lot_summary,
    merge_waypoint_states,
    SlotState,
    AERIAL_IOU_THRESHOLD,
    TOTAL_LOT_SPACES,
)


# -- Types ---------------------------------------------------------------------

Waypoint = Tuple[float, float, float]   # (x, y, z) in QLabs world space


# -- DroneScanner --------------------------------------------------------------

class DroneScanner:
    """
    Autonomous QDrone2 scanning controller for uParking.

    Responsibilities
    ----------------
    1. Fly the drone along a pre-planned waypoint path (A* / SNN output).
    2. At each waypoint: capture frame → detect → match occupancy.
    3. Maintain a persistent lot-wide occupancy map across waypoints.
    4. Broadcast live updates to the React dashboard via WebSocket.

    Parameters
    ----------
    qlabs_drone   : QLabs QDrone2 actor object (from your connection.py).
    ws_server     : WebSocket server object (from your existing ws module).
    model_path    : Path to fine-tuned YOLOv11 .pt weights file.
                    Pass None to use MockParkingDetector for development.
    iou_threshold : IoU threshold for aerial occupancy matching.
                    Default 0.15 per Peraza-Garzón et al. (2026).
    hover_time    : Seconds to hover at each waypoint before capturing.
                    Allows the drone to stabilise before taking the frame.
    total_spaces  : Total lot capacity.  156 for uParking simulation.

    Example
    -------
    >>> scanner = DroneScanner(drone, ws_server, model_path="best.pt")
    >>> waypoints = path_planner.get_waypoints()
    >>> asyncio.run(scanner.full_lot_scan(waypoints))
    """

    def __init__(
        self,
        qlabs_drone,
        ws_server,
        model_path: Optional[str] = None,
        iou_threshold: float = AERIAL_IOU_THRESHOLD,
        hover_time: float = 0.5,
        total_spaces: int = TOTAL_LOT_SPACES
    ):
        self.drone = qlabs_drone
        self.ws = ws_server
        self.iou_threshold = iou_threshold
        self.hover_time = hover_time
        self.total_spaces = total_spaces

        # Persistent lot-wide slot state — updated across all waypoints
        self._lot_states: Dict[int, SlotState] = {}
        self._slots_per_waypoint: int = 0  # set on first detection
        self._scan_log: List[dict] = []    # per-waypoint telemetry
        self._drone_pos: List[float] = [0.0, 0.0, 8.0]  # internal position tracker

        # Detector — use mock when no weights file available
        if model_path is None:
            print("[DroneScanner] No model_path provided — using MockParkingDetector")
            self.detector = MockParkingDetector(
                n_slots=12,
                occupancy_rate=0.5
            )
        else:
            self.detector = ParkingDetector(
                model_path=model_path,
                confidence=0.50   # Paper 2 deployment threshold
            )

    # -- Public API ------------------------------------------------------------

    async def full_lot_scan(self, waypoints: List[Waypoint]) -> Dict[int, SlotState]:
        """
        Execute a complete patrol scan of the parking lot.

        Iterates over all waypoints from the path planner, captures and
        processes a frame at each stop, and broadcasts live updates to the
        React dashboard.

        This implements the autonomous patrol concept described by Yan et al.
        (2024) — Paper 4 — combined with the detection pipeline from
        Peraza-Garzón et al. (2026) — Paper 2.

        Parameters
        ----------
        waypoints : Ordered list of (x, y, z) positions from A*/SNN planner.

        Returns
        -------
        Final lot-wide SlotState dict after all waypoints are visited.
        """
        print(f"[DroneScanner] Starting full scan: {len(waypoints)} waypoints")
        self._lot_states.clear()
        scan_start = time.time()

        # Slot ID offset: each waypoint covers a different zone so its
        # detected slot IDs are shifted to avoid collision in the global map
        id_offset = 0

        for wp_idx, position in enumerate(waypoints):
            print(f"[DroneScanner] Waypoint {wp_idx + 1}/{len(waypoints)} → {position}")

            # 1. Fly to waypoint using existing path planner output
            await self._fly_to(position)

            # 2. Hover briefly for camera stabilisation
            await asyncio.sleep(self.hover_time)

            # 3. Capture + process frame
            waypoint_states, telemetry = await self._scan_at_waypoint(
                wp_idx, position, id_offset
            )

            # 4. Merge this waypoint's detections into the full lot map
            if waypoint_states:
                if self._slots_per_waypoint == 0:
                    self._slots_per_waypoint = len(waypoint_states)

                merge_waypoint_states(
                    self._lot_states, waypoint_states, id_offset
                )
                id_offset += len(waypoint_states)

            # 5. Broadcast updated summary to React dashboard
            await self._broadcast_update(wp_idx, position, telemetry)

            self._scan_log.append(telemetry)

        elapsed = time.time() - scan_start
        print(f"[DroneScanner] Scan complete in {elapsed:.1f}s — "
              f"{len(self._lot_states)} slots mapped")

        return self._lot_states

    async def scan_single_waypoint(
        self,
        waypoint_id: int,
        position: Waypoint,
        id_offset: int = 0
    ) -> Optional[Dict[int, SlotState]]:
        """
        Scan a single waypoint without moving the drone.

        Useful for incremental scanning or re-scanning a specific zone
        after a state change is detected.

        Parameters
        ----------
        waypoint_id : Index of this waypoint in the patrol path.
        position    : (x, y, z) world position of the drone.
        id_offset   : Slot ID offset for global lot map placement.

        Returns
        -------
        SlotState dict for this waypoint, or None if frame was rejected.
        """
        states, telemetry = await self._scan_at_waypoint(
            waypoint_id, position, id_offset
        )
        if states:
            merge_waypoint_states(self._lot_states, states, id_offset)
            await self._broadcast_update(waypoint_id, position, telemetry)
        return states

    @property
    def lot_summary(self):
        """Current lot-wide occupancy summary (LotSummary dataclass)."""
        return build_lot_summary(self._lot_states, self.total_spaces)

    # -- Internal helpers ------------------------------------------------------

    async def _scan_at_waypoint(
        self,
        wp_idx: int,
        position: Waypoint,
        id_offset: int
    ) -> Tuple[Optional[Dict[int, SlotState]], dict]:
        """
        Capture one frame and run the full Paper 2 detection pipeline.

        Returns (slot_states, telemetry_dict).
        slot_states is None if the frame was rejected by blur check.
        """
        telemetry = {
            "waypoint_id": wp_idx,
            "position": position,
            "timestamp": time.time(),
            "frame_accepted": False,
            "vehicles_detected": 0,
            "slots_detected": 0,
            "occupied": 0,
            "available": 0,
            "occupancy_pct": 0.0,
            "laplacian": None,
        }

        # --- Step 1: capture frame from QDrone2 camera ---
        raw_frame = self._capture_frame()
        if raw_frame is None:
            print(f"[DroneScanner] WP {wp_idx}: no frame from camera")
            return None, telemetry

        # --- Step 2: preprocess (Paper 2 pipeline) ---
        prep = preprocess_frame(raw_frame)
        if prep is None:
            print(f"[DroneScanner] WP {wp_idx}: frame rejected (blur check failed)")
            return None, telemetry

        telemetry["frame_accepted"] = True
        telemetry["laplacian"] = round(prep.laplacian, 2)

        # --- Step 3: YOLOv11 dual-class detection (Paper 2) ---
        detection_result: DetectionResult = self.detector.detect(prep.frame)

        telemetry["vehicles_detected"] = detection_result.n_vehicles
        telemetry["slots_detected"]    = detection_result.n_slots

        # --- Step 4: IoU occupancy matching (Paper 2, Section 3.2.8) ---
        slot_states = match_occupancy(
            detection_result,
            iou_threshold=self.iou_threshold,
            waypoint_id=wp_idx
        )

        n_occ = sum(1 for s in slot_states.values() if s.occupied)
        telemetry["occupied"]      = n_occ
        telemetry["available"]     = len(slot_states) - n_occ
        telemetry["occupancy_pct"] = round(
            (n_occ / len(slot_states) * 100) if slot_states else 0.0, 2
        )

        return slot_states, telemetry

    async def _fly_to(self, position: Waypoint):
        """
        Fly QDrone2 to (x, y, z) using set_transform_and_dynamics().

        Adapted from the existing fly_to() in drone_scanner__7_.py.
        Interpolates in STEP_SIZE metre increments so QLabs is not
        overwhelmed with rapid-fire commands.  Runs blocking sleep on
        the thread executor so the asyncio event loop stays responsive.
        """
        if self.drone is None:
            await asyncio.sleep(0.05)   # mock / test mode
            return

        STEP_SIZE = 2.0     # metres per interpolation step
        STEP_DT   = 0.15    # seconds between steps

        x, y, z = position
        cx, cy, cz = self._drone_pos
        dx, dy, dz = x - cx, y - cy, z - cz
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        yaw  = math.atan2(dy, dx)

        loop = asyncio.get_event_loop()

        def _move(lx, ly, lz, rot_z):
            self.drone.set_transform_and_dynamics(
                location=[lx, ly, lz],
                rotation=[0, 0, rot_z],
                enableDynamics=False,
                waitForConfirmation=True
            )

        try:
            if dist < STEP_SIZE:
                await loop.run_in_executor(None, _move, x, y, z, 0)
            else:
                n_steps = max(int(dist / STEP_SIZE), 2)
                for step in range(1, n_steps + 1):
                    t  = step / n_steps
                    ix = cx + dx * t
                    iy = cy + dy * t
                    iz = cz + dz * t
                    await loop.run_in_executor(None, _move, ix, iy, iz, yaw)
                    await asyncio.sleep(STEP_DT)
        except Exception as e:
            print(f"[DroneScanner] fly_to warning: {e} — attempting single jump")
            try:
                await loop.run_in_executor(None, _move, x, y, z, 0)
            except Exception:
                pass
            await asyncio.sleep(1.0)

        self._drone_pos = [x, y, z]

    def _capture_frame(self) -> Optional[np.ndarray]:
        """
        Get a camera image from the QDrone2 downward-facing camera.

        QLabsQDrone2.get_image() returns (status, imageData) where
        imageData is a JPEG-encoded byte buffer.  We decode it into a
        BGR numpy array for OpenCV / preprocessing.py.

        Camera IDs on QDrone2:
            0 = front RGB camera
            3 = downward-facing camera  ← use this for top-down lot view

        Returns None on failure so the caller skips inference for this
        waypoint rather than crashing.
        """
        if self.drone is None:
            # Mock / test mode — return synthetic noise frame
            return (np.random.rand(480, 640, 3) * 255).astype(np.uint8)

        try:
            # camera 0 = front RGB (confirmed working in uParking)
            result = self.drone.get_image(0)
            if result[0] != True:
                print(f"[DroneScanner] Camera error — status {result[0]}")
                return None

            image_data = result[1]
            if not image_data:
                return None

            # Decode JPEG bytes → BGR numpy array
            buf   = np.frombuffer(image_data, dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)

            if frame is None:
                print("[DroneScanner] cv2.imdecode returned None")
                return None

            return frame

        except Exception as e:
            print(f"[DroneScanner] _capture_frame error: {e}")
            return None

    async def _broadcast_update(
        self,
        wp_idx: int,
        position: Waypoint,
        telemetry: dict
    ):
        """
        Broadcast current occupancy summary to the React dashboard.

        Payload structure matches what your existing WebSocket handler
        and React dashboard already expect, extended with Paper 2 fields.
        """
        summary = build_lot_summary(self._lot_states, self.total_spaces)

        payload = {
            # Drone telemetry
            "waypoint": wp_idx,
            "drone_position": {
                "x": position[0],
                "y": position[1],
                "z": position[2]
            },
            "timestamp": telemetry["timestamp"],

            # Detection stats for this waypoint
            "frame_accepted":    telemetry["frame_accepted"],
            "vehicles_detected": telemetry["vehicles_detected"],
            "slots_detected":    telemetry["slots_detected"],
            "laplacian":         telemetry["laplacian"],

            # Lot-wide occupancy — Paper 2 Equation 7
            "total_spaces":      summary.total_spaces,
            "occupied":          summary.occupied,
            "available":         summary.available,
            "unscanned":         summary.unscanned,
            "occupancy_pct":     summary.occupancy_pct,

            # Full per-slot map for dashboard parking grid render
            "slot_states":       summary.to_dict()["slot_states"]
        }

        try:
            # Replace with your actual WebSocket broadcast call, e.g.:
            # await self.ws.broadcast(json.dumps(payload))
            print(f"[WS] WP {wp_idx}: {summary.occupied} occupied / "
                  f"{summary.available} available / "
                  f"{summary.occupancy_pct:.1f}% full")
        except Exception as e:
            print(f"[DroneScanner] WebSocket broadcast failed: {e}")
