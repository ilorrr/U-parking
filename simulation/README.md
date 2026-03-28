# uParking Detection Pipeline
### Based on Peraza-Garzón et al. (2026) — Paper 2

---

## Files

| File | Purpose |
|------|---------|
| `preprocessing.py` | Blur detection, letterbox resize, normalization |
| `detector.py` | YOLOv11 dual-class inference (vehicle + parking slot) |
| `occupancy.py` | IoU matching, occupancy %, lot summary |
| `drone_scanner.py` | QDrone2 waypoint patrol loop — wires everything together |
| `test_pipeline.py` | End-to-end tests using MockParkingDetector (no GPU needed) |

---

## Install

```bash
pip install ultralytics opencv-python numpy
```

---

## Quick Start (no GPU / no weights file)

```bash
python test_pipeline.py
```

This runs all unit tests and a full simulated patrol scan using
`MockParkingDetector` — no QLabs connection or GPU required.

---

## Integration Into QLabs

Replace the two `TODO` stubs in `drone_scanner.py`:

```python
# In _fly_to():
await self.drone.move_to_position(*position, speed=1.5)

# In _capture_frame():
image_data = self.drone.get_image(0)
frame = np.frombuffer(image_data, dtype=np.uint8)
frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
return frame
```

Then instantiate with your actual QLabs objects:

```python
scanner = DroneScanner(
    qlabs_drone=my_qdrone2_actor,
    ws_server=my_websocket_server,
    model_path="path/to/best.pt"   # your fine-tuned YOLOv11 weights
)
waypoints = path_planner.get_waypoints()
await scanner.full_lot_scan(waypoints)
```

---

## Key Design Decisions (from Paper 2)

| Decision | Value | Reason |
|----------|-------|--------|
| IoU threshold | 0.15 | Aerial perspective distortion reduces bbox overlap |
| Confidence threshold | 0.50 | Paper 2 deployment recommendation |
| Input resolution | 640×640 | YOLOv11 training standard |
| Dual classes | vehicle + parking | Explicit empty-slot modeling improves accuracy |
| Blur filter | Laplacian variance ≥ 100 | Rejects motion-blurred drone frames |

---

## Citation

```
Peraza-Garzón, J.; Huerta-Mora, E.; Olivarría-González, M.; Quiñonez, Y.;
Rubio-Ayala, H.; Palacios-Navidad, J.A.; Peraza-Garzón, A.
Intelligent Car Park Occupancy Monitoring System Based on Parking Slot and
Vehicle Detection Using DJI Mini 3 Aerial Imagery and YOLOv11.
AI 2026, 7, 74. https://doi.org/10.3390/ai7020074
```
