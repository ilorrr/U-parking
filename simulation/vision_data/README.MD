# UParking — Autonomous Parking Lot Management System

## Quanser QLabs Simulation

### Senior Design Project — TAMIU CSCE

---

## Overview

UParking is an autonomous parking lot management system that uses QDrone2 drones in the Quanser QLabs simulation environment to scan a 320-spot parking lot across 2 sections and detect vehicle occupancy. The system features a multi-drone parallel scanning engine, an adaptive occupancy learning model, and a vision-based classification pipeline for training a CNN on aerial parking spot images.


## Architecture

The simulation runs on a single QLabs instance with:

- **3 QDrone2 actors** — parallel zone-based occupancy scanning
- **10 QCar2 actors** — simulated vehicles entering and parking via Hungarian algorithm assignment
- **15–35 static trucks** — randomized per run for dataset diversity
- **320 parking spots** — 2 sections × 4 rows × 40 columns
- **Django REST backend** — receives occupancy updates for the React dashboard and mobile app

---

## Known Limitation: QDrone2 Camera After Parallel Scanning

### The Problem

The QDrone2's `get_image()` API becomes permanently unresponsive after a multi-drone parallel scan. This is a limitation of the QLabs communication protocol, not a bug in our code.

**Root cause:** During the parallel scan, each drone calls `set_transform_and_dynamics(waitForConfirmation=False)` hundreds of times across multiple threads. Each call generates an ACK response from QLabs that is never consumed (since `waitForConfirmation=False` skips reading the response). These orphaned ACK packets accumulate in the shared TCP socket's receive buffer. When `get_image()` is subsequently called, it reads a stale ACK packet instead of the expected image response, causing a communication timeout.

**What we tried (all failed after parallel scan):**

| Approach | Result |
|----------|--------|
| `flush_receive()` × 50 | Buffer too deep, stale packets remain |
| Restart RT model | RT model restarts but pipe stays corrupted |
| `enableDynamics=True` | Camera still reads stale ACKs |
| `waitForConfirmation=True` | Hangs indefinitely (blocked by stale data) |
| `set_velocity_and_request_state` | Also times out on corrupted pipe |
| Respawn drone actor | Works but unrealistic (not applicable to real hardware) |
| Second QLabs connection | Image responses routed through RT model session on first connection |
| Separate drone 0 from scan (drones 1+2 only) | All drones share same TCP socket — pipe corruption affects all actors |

**What works:**

| Approach | Result |
|----------|--------|
| `get_image()` before any parallel scan | 147/147 captures, 0 failures |
| QLabsFreeCamera after parallel scan | 147/147 captures, 0 failures |

**Key insight:** The QDrone2 camera works perfectly when the communication pipe is clean. The issue is exclusively caused by the accumulation of unconsumed ACK packets from `set_transform_and_dynamics(waitForConfirmation=False)` during multi-threaded scanning.

### Our Solution: QLabsFreeCamera for Training Data Collection

We use a `QLabsFreeCamera` (actor 99) to capture training images. The FreeCamera is a separate QLabs actor with its own clean communication channel — it is unaffected by the drone pipe corruption.

**Why the training data is valid:**

The FreeCamera is positioned at the exact same 3D coordinates and altitude as the drone would be at each parking spot. It captures the same scene, same vehicles, same lighting, same perspective. The CNN training pipeline receives identical pixel content regardless of which actor type captured the image. The model learns to distinguish "colored vehicle top in a parking spot" from "gray pavement with white lines" — this visual pattern is actor-independent.

**How it maps to real hardware:**

On a real QDrone2, the camera subsystem (Intel RealSense + downward camera) operates independently from the flight controller. The `get_image()` API limitation is specific to the QLabs simulation's shared TCP socket architecture. Real drone hardware uses dedicated camera buses (CSI/MIPI) that are physically separate from the flight controller's communication channel. A model trained on FreeCamera data will transfer directly to real drone camera data with minimal fine-tuning, because the visual features (vehicle shapes, pavement textures, parking line patterns) are scene properties — not camera properties.

---

## Pipeline Flow

```
1. Setup
   └── Spawn scene: buildings, parking lot (320 spots), vehicles, 3 drones

2. Vision Collection (FreeCamera — clean pipe)
   ├── IoU pre-scan labels all spots as occupied/empty (instant)
   ├── FreeCamera spawns at drone altitude, looking straight down
   ├── Teleports to each spot, captures 640×480 frame
   ├── Saves to vision_data/occupied/ and vision_data/empty/
   └── Result: labeled training dataset (147+ images per run)

3. Parallel Scan (3 QDrone2s — fast occupancy)
   ├── Spots split into 3 geographically contiguous zones
   ├── Each drone scans its zone via set_transform_and_dynamics
   ├── Occupancy detected via IoU + distance fallback
   └── Result: full lot occupancy in ~87 seconds

4. Learning Model
   ├── Records scan results (25+ historical runs)
   ├── Computes per-spot occupancy probability
   ├── Identifies high-risk (≥70%) and low-risk (≤10%) spots
   ├── Detects anomalies (permit holders, consistently empty)
   └── Exports heatmap data and scan reports

5. CNN Training (offline)
   ├── Loads images from vision_data/
   ├── 4-layer CNN: Conv(3→16→32→64→128) + FC(128→64→1)
   ├── Data augmentation: flips, brightness, rotation
   ├── Binary classification: P(occupied) with sigmoid output
   └── Outputs: vision_model.pth + evaluation metrics

6. Backend Integration
   ├── POST /api/occupancy-update/ pushes scan results to Django
   ├── ParkingSpot model tracks per-spot occupancy state
   ├── React dashboard (Lot.jsx, Dashboard.jsx) displays live map
   └── React Native app shows availability to drivers
```

---

## File Structure

### Core Simulation

| File | Purpose |
|------|---------|
| `main.py` | Entry point — orchestrates full pipeline |
| `qlabs_connect.py` | Connect to or launch QLabs instance |
| `parking_lot.py` | Generate 320-spot grid, spawn line markers |
| `occupancy.py` | IoU + distance-based occupancy detection |
| `qcar_spawner.py` | Spawn and drive QCar2 vehicles to spots |
| `parking_assignment.py` | Hungarian algorithm for optimal car-to-spot assignment |
| `drone_scanner.py` | Single-drone scanner with QCar poller |
| `multi_drone_scanner.py` | 3-drone parallel scanner + vision collection pass |
| `occupancy_learning.py` | Historical learning model, risk classification, reports |

### Path Planning

| File | Purpose |
|------|---------|
| `path_planner.py` | A*, Dijkstra, BFS, Hill-Climbing, GA, SA strategies |
| `path_planner_integration.py` | Bridges path planner to lot layout |

### Vision Pipeline

| File | Purpose |
|------|---------|
| `vision_collector.py` | Captures and saves labeled training images |
| `vision_model.py` | CNN architecture, training, and evaluation |
| `vision_detector.py` | Inference module with hybrid IoU+CNN detection |
| `live_scanner.py` | Velocity-based scanner with streaming callback |

### YOLOv11 Detection (Paper 2)

| File | Purpose |
|------|---------|
| `uparking_detection/detector.py` | YOLOv11 dual-class detector (vehicle + parking) |
| `uparking_detection/preprocessing.py` | Blur detection, letterbox resize, normalization |
| `uparking_detection/occupancy.py` | Aerial IoU matching at 0.15 threshold |
| `uparking_detection/drone_scanner.py` | Full detection pipeline with WebSocket broadcast |

---

## Training the Vision Model

### Step 1: Collect Data

Run `main.py` multiple times with different vehicle configurations. Each run captures ~150 labeled images via the FreeCamera.

```bash
python main.py    # run 5-6 times for ~750-900 images
```

Images are saved to:
```
vision_data/
├── occupied/     # vehicle visible in spot
│   ├── S1-A0_00000.jpg
│   ├── S1-A1_00001.jpg
│   └── ...
├── empty/        # pavement only
│   ├── S1-A6_00000.jpg
│   ├── S1-A10_00001.jpg
│   └── ...
└── metadata.json
```

### Step 2: Train

```bash
pip install torch torchvision
python vision_model.py --data vision_data --epochs 30
```

Output:
```
  [Model] Dataset: 750 samples (350 occupied, 400 empty)
  [Model] Device: cuda
  [Model] Train: 600, Val: 150
  [Model] Parameters: 103,297
  Epoch   1/30  |  Train loss: 0.6821  |  Val loss: 0.5432  |  Val acc: 72.0%
  Epoch  30/30  |  Train loss: 0.0234  |  Val loss: 0.0891  |  Val acc: 95.3%
```

### Step 3: Evaluate

```bash
python vision_model.py --data vision_data --evaluate
```

### Step 4: Deploy

The trained model integrates via `vision_detector.py`:

```python
from vision_detector import HybridDetector

detector = HybridDetector(
    drone, model_path="vision_model.pth",
    strategy="vision_primary"   # CNN first, IoU fallback
)
occupied, method, confidence = detector.predict(spot, static_pos, live_pos)
```

---

## Real-World Deployment Path

The QDrone2 hardware runs an NVIDIA Jetson Xavier NX with 384 CUDA cores and 48 Tensor cores. The trained CNN converts to TensorRT and runs onboard at the edge — no ground station processing needed.

```
Simulation (current)          →  Real Hardware
─────────────────────────────────────────────────
FreeCamera capture            →  Onboard downward camera (CSI)
set_transform_and_dynamics    →  GPS waypoint navigation (DJI SDK)
IoU coordinate detection      →  CNN/YOLOv11 vision detection
Django REST POST              →  WiFi POST from Jetson to backend
QLabs TCP socket              →  Dedicated camera bus (no pipe issue)
```

The QLabs camera limitation does not exist on real hardware. The Jetson Xavier NX accesses cameras through dedicated CSI/MIPI buses that are physically separate from the flight controller's communication channel.

---

## Dependencies

```bash
pip install numpy opencv-python scipy
pip install torch torchvision          # for CNN training
pip install ultralytics                # for YOLOv11 (optional)
```

Quanser QLabs and QUARC must be installed separately per Quanser's license.

---

## References

1. Kuhn, H.W. (1955). "The Hungarian Method for the assignment problem." Naval Research Logistics Quarterly, 2(1-2), 83-97.

2. Peraza-Garzón, J. et al. (2026). "Intelligent Car Park Occupancy Monitoring System Based on Parking Slot and Vehicle Detection Using DJI Mini 3 Aerial Imagery and YOLOv11." AI 2026, 7, 74.

3. Hsu, T.L. et al. (2017). "Drone-based Object Counting by Spatially Regularized Regional Proposal Networks." CARPK Dataset — 89,777 drone-view car images.

4. Amato, G. et al. (2017). "Deep learning for decentralized parking lot occupancy detection." CNRPark+EXT Dataset — 150,000 labeled parking patches.
