"""
parking_assignment.py
---------------------
Optimal parking spot assignment using the Hungarian Algorithm.
"""

import math
import numpy as np
from scipy.optimize import linear_sum_assignment


# ── Core assignment function ──────────────────────────────────────────────────

def build_cost_matrix(car_positions, free_spots):
    """
    Build an N x M cost matrix where cost[i][j] is the Euclidean distance
    from car i's spawn position to spot j's centre.
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
    """
    if not car_positions or not free_spots:
        return []

    n_cars  = len(car_positions)
    n_spots = len(free_spots)

    cost = build_cost_matrix(car_positions, free_spots)

    if n_cars <= n_spots:
        row_ind, col_ind = linear_sum_assignment(cost)
        assignments = [free_spots[col_ind[i]] for i in range(len(row_ind))]
    else:
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
    """
    hungarian_spots   = assign_spots_hungarian(car_positions, free_spots)
    hungarian_metrics = compute_assignment_metrics(car_positions, hungarian_spots)

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
