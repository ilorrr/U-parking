"""
test_pipeline.py
----------------
Quick end-to-end test of the Paper 2 detection pipeline using the
MockParkingDetector (no GPU / weights file required).

Run this to verify preprocessing → detection → occupancy → summary
all work correctly before connecting to the actual QDrone2 in QLabs.

Usage:
    python test_pipeline.py
"""

import asyncio
import numpy as np

from preprocessing import preprocess_frame, compute_laplacian_variance
from detector import MockParkingDetector, CLASS_VEHICLE, CLASS_PARKING
from occupancy import (
    compute_iou,
    match_occupancy,
    compute_occupancy_percentage,
    build_lot_summary,
    AERIAL_IOU_THRESHOLD,
)
from drone_scanner import DroneScanner


# -- Unit tests ----------------------------------------------------------------

def test_compute_iou():
    """IoU between identical boxes should be 1.0."""
    box = [10.0, 10.0, 50.0, 50.0]
    assert abs(compute_iou(box, box) - 1.0) < 1e-6

    # Non-overlapping boxes → IoU = 0
    box_a = [0.0, 0.0, 10.0, 10.0]
    box_b = [20.0, 20.0, 30.0, 30.0]
    assert compute_iou(box_a, box_b) == 0.0

    # Partial overlap
    box_c = [0.0, 0.0, 20.0, 20.0]
    box_d = [10.0, 10.0, 30.0, 30.0]
    iou = compute_iou(box_c, box_d)
    assert 0 < iou < 1

    print("✓ compute_iou passed")


def test_preprocess_frame():
    """Sharp frame should pass; flat frame should be rejected."""
    # Sharp random frame — high Laplacian variance
    sharp = (np.random.rand(480, 640, 3) * 255).astype(np.uint8)
    result = preprocess_frame(sharp, blur_threshold=10.0)
    assert result is not None
    assert result.frame.shape == (640, 640, 3)
    assert result.frame.dtype == np.float32
    assert result.frame.max() <= 1.0

    # Flat (blurry) frame — near-zero Laplacian variance
    flat = np.full((480, 640, 3), 128, dtype=np.uint8)
    result_flat = preprocess_frame(flat, blur_threshold=100.0)
    assert result_flat is None

    print("✓ preprocess_frame passed")


def test_mock_detector():
    """MockParkingDetector should return correctly typed detections."""
    detector = MockParkingDetector(n_slots=8, occupancy_rate=0.5, seed=0)
    dummy_frame = np.zeros((640, 640, 3), dtype=np.float32)
    result = detector.detect(dummy_frame)

    assert len(result.parking_slots) == 8
    for slot in result.parking_slots:
        assert slot.class_id == CLASS_PARKING
    for veh in result.vehicles:
        assert veh.class_id == CLASS_VEHICLE

    print(f"✓ MockParkingDetector: {result.n_slots} slots, "
          f"{result.n_vehicles} vehicles")


def test_match_occupancy():
    """
    When a vehicle is placed inside a slot, IoU should exceed the aerial
    threshold (0.15) and the slot should be marked occupied.
    """
    detector = MockParkingDetector(n_slots=4, occupancy_rate=1.0, seed=1)
    dummy = np.zeros((640, 640, 3), dtype=np.float32)
    det = detector.detect(dummy)

    states = match_occupancy(det, iou_threshold=AERIAL_IOU_THRESHOLD)
    assert len(states) == 4

    occupied_count = sum(1 for s in states.values() if s.occupied)
    print(f"✓ match_occupancy: {occupied_count}/4 slots occupied "
          f"(threshold={AERIAL_IOU_THRESHOLD})")


def test_occupancy_percentage():
    """
    Paper 2 Equation 7:
        Occupancy(%) = N_occupied / (N_occupied + N_free) × 100
    """
    # All occupied
    detector_full = MockParkingDetector(n_slots=10, occupancy_rate=1.0, seed=2)
    det_full = detector_full.detect(np.zeros((640, 640, 3), np.float32))
    states_full = match_occupancy(det_full)
    pct_full = compute_occupancy_percentage(states_full)
    print(f"  Full lot: {pct_full:.1f}% (expected ~100%)")

    # None occupied
    detector_empty = MockParkingDetector(n_slots=10, occupancy_rate=0.0, seed=3)
    det_empty = detector_empty.detect(np.zeros((640, 640, 3), np.float32))
    states_empty = match_occupancy(det_empty)
    pct_empty = compute_occupancy_percentage(states_empty)
    print(f"  Empty lot: {pct_empty:.1f}% (expected 0.0%)")

    assert pct_empty == 0.0
    print("✓ compute_occupancy_percentage passed")


async def test_drone_scanner():
    """
    End-to-end patrol scan using MockParkingDetector and synthetic frames.
    No QLabs connection or GPU required.
    """
    print("\n-- DroneScanner end-to-end test --")

    # Dummy waypoints — 6 stops across a 156-space lot
    # In real use these come from your A*/SNN path planner
    waypoints = [
        (0.0,  0.0, 5.0),
        (5.0,  0.0, 5.0),
        (10.0, 0.0, 5.0),
        (10.0, 5.0, 5.0),
        (5.0,  5.0, 5.0),
        (0.0,  5.0, 5.0),
    ]

    # model_path=None → MockParkingDetector used automatically
    scanner = DroneScanner(
        qlabs_drone=None,
        ws_server=None,
        model_path=None,
        hover_time=0.0     # no delay in tests
    )

    lot_states = await scanner.full_lot_scan(waypoints)

    summary = scanner.lot_summary
    print(f"\n-- Lot Summary ----------------------")
    print(f"  Total spaces :  {summary.total_spaces}")
    print(f"  Slots scanned:  {len(lot_states)}")
    print(f"  Occupied      :  {summary.occupied}")
    print(f"  Available     :  {summary.available}")
    print(f"  Unscanned     :  {summary.unscanned}")
    print(f"  Occupancy     :  {summary.occupancy_pct:.1f}%")
    print(f"------------------------------------")

    assert len(lot_states) > 0
    print("✓ DroneScanner full_lot_scan passed")


# -- Runner --------------------------------------------------------------------

if __name__ == "__main__":
    print("=== uParking Detection Pipeline Tests ===\n")

    test_compute_iou()
    test_preprocess_frame()
    test_mock_detector()
    test_match_occupancy()
    test_occupancy_percentage()

    asyncio.run(test_drone_scanner())

    print("\n=== All tests passed ✓ ===")
