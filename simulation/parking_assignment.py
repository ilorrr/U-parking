"""
parking_assignment.py
---------------------
Optimal parking spot assignment using the Hungarian Algorithm.

The Hungarian Algorithm (Kuhn-Munkres, 1955) solves the assignment problem:
given N cars and M spots, find the assignment that minimises total travel
distance across all cars simultaneously.

This is provably optimal - no other assignment can produce a lower total
distance.  Time complexity O(n^3), practical for lots up to ~500 spots.

Reference:
    Kuhn, H.W. (1955). "The Hungarian Method for the assignment problem."
    Naval Research Logistics Quarterly, 2(1-2), 83-97.

Integration with uParking:
    Called by qcar_spawner.py instead of _pick_spots_by_row().
    Takes car spawn positions and free spot positions, returns the
    optimal (car -> spot) pairing.

Usage:
    from parking_assignment import assign_spots_hungarian

    assignments = assign_spots_hungarian(
        car_positions = [(x1,y1), (x2,y2), ...],
        free_spots    = [spot_dict, spot_dict, ...],
    )
    # assignments[i] = spot_dict assigned to car i
"""

import math
import numpy as np
from scipy.optimize import linear_sum_assignment


# ── Core assignment function ──────────────────────────────────────────────────

def build_cost_matrix(car_positions, free_spots):
    """
    Build an N x M cost matrix where cost[i][j] is the Euclidean distance
    from car i's spawn position to spot j's centre.

    Parameters
    ----------
    car_positions : list of (x, y) tuples — one per incoming QCar
    free_spots    : list of spot dicts with "center": [x, y, z]

    Returns
    -------
    numpy array of shape (N, M), dtype float64
    """
    n = len(car_positions)
    m = len(free_spots)
    cost = np.zeros((n, m), dtype=np.float64)

    for i, (cx, cy) in enumerate(car_positions):
        for j, spot in enumerate(free_spots):
            sx, sy = spot["center"][0], spot["center"][1]
            cost[i, j] = math.hypot(cx - sx, cy - sy)

    return cost


def assign_spots_hungarian(car_positions, free_spots):
    """
    Assign each car to a free spot using the Hungarian Algorithm to
    minimise total travel distance across all cars.

    If there are more cars than spots, only as many cars as spots
    will receive an assignment.  If there are more spots than cars,
    only the closest spots will be used.

    Parameters
    ----------
    car_positions : list of (x, y) — spawn positions of incoming QCars
    free_spots    : list of spot dicts from build_parking_lot()

    Returns
    -------
    list of spot dicts, one per car, in the same order as car_positions.
    Returns empty list if either input is empty.

    Example
    -------
    >>> positions = [(10, -20), (10, -23.5), (10, -27)]
    >>> chosen = assign_spots_hungarian(positions, free_spots)
    >>> for i, spot in enumerate(chosen):
    ...     print(f"Car {i} -> {spot['label']}")
    """
    if not car_positions or not free_spots:
        return []

    n_cars  = len(car_positions)
    n_spots = len(free_spots)

    cost = build_cost_matrix(car_positions, free_spots)

    # scipy linear_sum_assignment requires n <= m (more cols than rows)
    # If more cars than spots, transpose and solve the other way
    if n_cars <= n_spots:
        row_ind, col_ind = linear_sum_assignment(cost)
        assignments = [free_spots[col_ind[i]] for i in range(len(row_ind))]
    else:
        # More cars than spots — assign all spots, leave excess cars unassigned
        row_ind, col_ind = linear_sum_assignment(cost[:n_spots, :])
        assignments = [None] * n_cars
        for r, c in zip(row_ind, col_ind):
            assignments[r] = free_spots[c]
        assignments = [a for a in assignments if a is not None]

    return assignments


# ── Metrics ───────────────────────────────────────────────────────────────────

def compute_assignment_metrics(car_positions, assigned_spots):
    """
    Compute travel distance metrics for an assignment.

    Parameters
    ----------
    car_positions  : list of (x, y) tuples
    assigned_spots : list of spot dicts (same order as car_positions)

    Returns
    -------
    dict with keys:
        total_distance   : sum of all travel distances (metres)
        mean_distance    : average travel distance per car
        max_distance     : longest single trip
        min_distance     : shortest single trip
        distances        : list of individual distances
    """
    distances = []
    for (cx, cy), spot in zip(car_positions, assigned_spots):
        sx, sy = spot["center"][0], spot["center"][1]
        distances.append(math.hypot(cx - sx, cy - sy))

    return {
        "total_distance" : sum(distances),
        "mean_distance"  : sum(distances) / len(distances) if distances else 0,
        "max_distance"   : max(distances) if distances else 0,
        "min_distance"   : min(distances) if distances else 0,
        "distances"      : distances,
    }


def compare_with_greedy(car_positions, free_spots):
    """
    Compare Hungarian assignment against simple greedy nearest-neighbour.

    Greedy: each car sequentially takes the nearest remaining spot.
    Hungarian: globally optimal assignment computed in one shot.

    Returns
    -------
    dict with 'hungarian' and 'greedy' metric dicts, plus 'saving_pct'
    showing how much total distance Hungarian saves over greedy.
    """
    # Hungarian
    hungarian_spots   = assign_spots_hungarian(car_positions, free_spots)
    hungarian_metrics = compute_assignment_metrics(car_positions, hungarian_spots)

    # Greedy nearest-neighbour
    remaining = list(free_spots)
    greedy_spots = []
    for cx, cy in car_positions:
        if not remaining:
            break
        nearest = min(remaining,
                      key=lambda s: math.hypot(cx - s["center"][0],
                                               cy - s["center"][1]))
        greedy_spots.append(nearest)
        remaining.remove(nearest)

    greedy_metrics = compute_assignment_metrics(car_positions, greedy_spots)

    h_total = hungarian_metrics["total_distance"]
    g_total = greedy_metrics["total_distance"]
    saving  = g_total - h_total
    saving_pct = (saving / g_total * 100) if g_total > 0 else 0

    return {
        "hungarian"   : hungarian_metrics,
        "greedy"      : greedy_metrics,
        "saving_m"    : saving,
        "saving_pct"  : saving_pct,
    }


# ── Quick test ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import random
    random.seed(42)

    # Simulate 10 car spawn positions (staggered along Y=-20)
    car_positions = [
        (10.0, -20.0 + i * 3.5)
        for i in range(10)
    ]

    # Simulate 156 parking spots in a 4x39 grid
    mock_spots = []
    for row in range(4):
        for col in range(39):
            cx = -4.0 - row * 15.5
            cy = -10.0 + col * 2.7
            mock_spots.append({
                "label"    : f"{'ABCD'[row]}{col}",
                "center"   : [cx, cy, 0.005],
                "row"      : row,
                "col"      : col,
                "direction": "right" if row % 2 == 0 else "left",
            })

    print("=== Hungarian Algorithm Parking Assignment ===\n")

    # Run assignment
    assigned = assign_spots_hungarian(car_positions, mock_spots)
    metrics  = compute_assignment_metrics(car_positions, assigned)

    print("Assignments:")
    for i, (pos, spot) in enumerate(zip(car_positions, assigned)):
        dist = metrics["distances"][i]
        print(f"  Car {i} at {pos} -> {spot['label']}  ({dist:.1f}m)")

    print(f"\nTotal distance   : {metrics['total_distance']:.1f}m")
    print(f"Mean per car     : {metrics['mean_distance']:.1f}m")
    print(f"Max single trip  : {metrics['max_distance']:.1f}m")

    print("\n--- Comparison with Greedy ---")
    comparison = compare_with_greedy(car_positions, mock_spots)
    print(f"Hungarian total  : {comparison['hungarian']['total_distance']:.1f}m")
    print(f"Greedy total     : {comparison['greedy']['total_distance']:.1f}m")
    print(f"Distance saved   : {comparison['saving_m']:.1f}m  "
          f"({comparison['saving_pct']:.1f}%)")

    print("\n=== All tests passed ===")
