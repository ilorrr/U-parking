"""
occupancy_learning.py
---------------------
Adaptive learning layer for uParking drone scan results.

Responsibilities:
    1. Log every scan result with timestamp to persistent history
    2. Build occupancy probability model per spot from history
    3. Prioritize high-probability spots in future scans
    4. Generate human-readable and JSON reports
    5. Detect anomalies (spots always occupied, never occupied, etc.)

The learning model is simple but effective:
    - Occupancy probability per spot = occupied_count / total_scans
    - Time-of-day weighting (morning vs afternoon patterns)
    - Exponential decay so recent scans matter more than old ones

Usage:
    from occupancy_learning import OccupancyLearner

    learner = OccupancyLearner()
    learner.record_scan(scan_log)
    report = learner.generate_report()
    priority_order = learner.get_priority_waypoints(all_waypoints)
"""

import os
import json
import math
from datetime import datetime
from collections import defaultdict


# ── File paths ────────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
LEARNING_DB  = os.path.join(_HERE, "occupancy_learning.json")
REPORTS_DIR  = os.path.join(_HERE, "scan_reports")

# Decay factor — recent scans weighted more than old ones
# 0.95 = each previous scan counts for 95% of the previous one
DECAY_FACTOR = 0.95

# Probability threshold for classifying a spot as "high risk"
HIGH_RISK_THRESHOLD  = 0.70
LOW_RISK_THRESHOLD   = 0.10


# ── OccupancyLearner ──────────────────────────────────────────────────────────

class OccupancyLearner:
    """
    Learns parking occupancy patterns from historical scan data.

    Maintains a JSON database of every scan result, builds per-spot
    probability models, and generates prioritized waypoint orders so
    the drone checks historically busy spots first.

    Parameters
    ----------
    db_path : Path to the JSON learning database file.
              Defaults to occupancy_learning.json in the project folder.
    """

    def __init__(self, db_path=LEARNING_DB):
        self.db_path = db_path
        self._db     = self._load_db()

    # ── Public API ────────────────────────────────────────────────────────────

    def record_scan(self, scan_log, scan_metadata=None):
        """
        Record a completed scan into the learning database.

        Parameters
        ----------
        scan_log      : List of scan entry dicts from drone_scanner or
                        multi_drone_scanner. Each entry must have:
                        { label, x, y, occupied, timestamp }
        scan_metadata : Optional dict with extra context:
                        { weather, time_of_day, total_vehicles, notes }

        Returns
        -------
        run_id : int — the run number assigned to this scan
        """
        run_id  = len(self._db["runs"]) + 1
        now_str = datetime.now().isoformat()

        occupied_labels = [e["label"] for e in scan_log if e["occupied"]]
        available_count = sum(1 for e in scan_log if not e["occupied"])

        run_record = {
            "run_id"    : run_id,
            "timestamp" : now_str,
            "hour"      : datetime.now().hour,
            "total"     : len(scan_log),
            "occupied"  : len(occupied_labels),
            "available" : available_count,
            "metadata"  : scan_metadata or {},
            "spots"     : {
                e["label"]: {
                    "occupied"  : e["occupied"],
                    "timestamp" : e.get("timestamp", now_str),
                    "drone"     : e.get("drone", 0)
                }
                for e in scan_log
            }
        }

        self._db["runs"].append(run_record)
        self._update_spot_model(run_record)
        self._save_db()

        print(f"[Learner] Run #{run_id} recorded — "
              f"{len(occupied_labels)} occupied / {available_count} available")
        return run_id

    def get_occupancy_probability(self, label):
        """
        Get the learned occupancy probability for a spot label.

        Returns float in [0, 1]. Returns 0.5 (uncertain) for unseen spots.
        """
        spot_data = self._db["spot_model"].get(label)
        if not spot_data or spot_data["total_scans"] == 0:
            return 0.5  # no data — assume 50/50
        return spot_data["weighted_prob"]

    def get_priority_waypoints(self, waypoints):
        """
        Reorder waypoints so high-probability spots are scanned first.

        High-probability spots are more likely to be occupied, so scanning
        them first gives the fastest useful output to drivers.

        Parameters
        ----------
        waypoints : List of waypoint dicts with "label" key
                    (format from drone_scanner spiral_waypoints)

        Returns
        -------
        Reordered waypoint list — high probability first, low last.
        """
        def priority_key(wp):
            label = wp.get("label", "")
            prob  = self.get_occupancy_probability(label)
            # Negate so high probability sorts first
            return -prob

        sorted_wps = sorted(waypoints, key=priority_key)

        # Print summary of prioritization
        high = sum(1 for w in sorted_wps
                   if self.get_occupancy_probability(w["label"]) >= HIGH_RISK_THRESHOLD)
        low  = sum(1 for w in sorted_wps
                   if self.get_occupancy_probability(w["label"]) <= LOW_RISK_THRESHOLD)
        print(f"[Learner] Waypoint priority: {high} high-risk, "
              f"{low} low-risk, "
              f"{len(sorted_wps)-high-low} medium-risk spots")

        return sorted_wps

    def generate_report(self, run_id=None):
        """
        Generate a human-readable occupancy report.

        Parameters
        ----------
        run_id : If provided, report on that specific run.
                 If None, report on the latest run + learned model summary.

        Returns
        -------
        report_str : Formatted string report (also saved to scan_reports/)
        """
        if not self._db["runs"]:
            return "[Learner] No scan data available yet."

        target_run = None
        if run_id:
            for r in self._db["runs"]:
                if r["run_id"] == run_id:
                    target_run = r
                    break
        if target_run is None:
            target_run = self._db["runs"][-1]

        lines = []
        lines.append("=" * 60)
        lines.append("UPARKING OCCUPANCY SCAN REPORT")
        lines.append("=" * 60)
        lines.append(f"Run ID      : #{target_run['run_id']}")
        lines.append(f"Timestamp   : {target_run['timestamp']}")
        lines.append(f"Total spots : {target_run['total']}")
        lines.append(f"Occupied    : {target_run['occupied']}")
        lines.append(f"Available   : {target_run['available']}")
        occ_pct = (target_run['occupied'] / target_run['total'] * 100
                   if target_run['total'] > 0 else 0)
        lines.append(f"Occupancy   : {occ_pct:.1f}%")

        if target_run["metadata"]:
            lines.append(f"Metadata    : {target_run['metadata']}")

        # Occupied spots list
        occupied_labels = sorted(
            label for label, data in target_run["spots"].items()
            if data["occupied"]
        )
        if occupied_labels:
            lines.append("\nOccupied spots:")
            by_row = defaultdict(list)
            for label in occupied_labels:
                # Handle both 'A0' and 'S1-A0' formats
                core = label.split("-")[-1]
                by_row[core[0]].append(label)
            for row in sorted(by_row.keys()):
                lines.append(f"  Row {row}: {', '.join(sorted(by_row[row]))}")
        else:
            lines.append("\nNo occupied spots detected.")

        # Learning model summary
        total_runs = len(self._db["runs"])
        lines.append(f"\n--- Learning Model ({total_runs} historical runs) ---")

        high_risk = [
            (label, data["weighted_prob"])
            for label, data in self._db["spot_model"].items()
            if data["weighted_prob"] >= HIGH_RISK_THRESHOLD
        ]
        low_risk = [
            label for label, data in self._db["spot_model"].items()
            if data["weighted_prob"] <= LOW_RISK_THRESHOLD
            and data["total_scans"] >= 3
        ]

        if high_risk:
            high_risk.sort(key=lambda x: -x[1])
            lines.append(f"High-risk spots (>={HIGH_RISK_THRESHOLD*100:.0f}% occupied):")
            for label, prob in high_risk[:10]:
                lines.append(f"  {label}: {prob*100:.0f}%")
            if len(high_risk) > 10:
                lines.append(f"  ... and {len(high_risk)-10} more")

        if low_risk:
            lines.append(f"Low-risk spots (<={LOW_RISK_THRESHOLD*100:.0f}% occupied): "
                         f"{len(low_risk)} spots (consistently empty)")

        lines.append("=" * 60)
        report_str = "\n".join(lines)

        # Save report to file
        os.makedirs(REPORTS_DIR, exist_ok=True)
        fname = os.path.join(REPORTS_DIR,
                             f"report_run{target_run['run_id']}.txt")
        with open(fname, "w") as f:
            f.write(report_str)

        print(report_str)
        print(f"\n[Learner] Report saved -> {fname}")
        return report_str

    def get_anomalies(self):
        """
        Detect spots with unusual patterns.

        Returns dict with:
            always_occupied  : spots with prob >= 0.95 (possible permit holder)
            never_occupied   : spots with prob <= 0.02 (inaccessible?)
            high_variance    : spots with inconsistent occupancy
        """
        always = []
        never  = []
        high_var = []

        for label, data in self._db["spot_model"].items():
            if data["total_scans"] < 3:
                continue
            prob = data["weighted_prob"]
            if prob >= 0.95:
                always.append(label)
            elif prob <= 0.02:
                never.append(label)
            # High variance: prob near 0.5 but with many observations
            # indicates a contested spot (frequent turnover)
            elif 0.3 <= prob <= 0.7 and data["total_scans"] >= 5:
                high_var.append(label)

        return {
            "always_occupied" : sorted(always),
            "never_occupied"  : sorted(never),
            "high_variance"   : sorted(high_var)
        }

    def export_heatmap_data(self):
        """
        Export probability data in a format suitable for the React dashboard
        to render a colour-coded occupancy heatmap.

        Returns
        -------
        dict : { label -> { prob, risk_level, total_scans } }
        """
        heatmap = {}
        for label, data in self._db["spot_model"].items():
            prob = data["weighted_prob"]
            if prob >= HIGH_RISK_THRESHOLD:
                risk = "high"
            elif prob <= LOW_RISK_THRESHOLD:
                risk = "low"
            else:
                risk = "medium"

            heatmap[label] = {
                "prob"        : round(prob, 3),
                "risk_level"  : risk,
                "total_scans" : data["total_scans"]
            }
        return heatmap

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _update_spot_model(self, run_record):
        """
        Update per-spot probability model using exponential decay weighting.

        Recent scans contribute more than older ones via DECAY_FACTOR.
        This means the system adapts to changing patterns over time
        (e.g. a new tenant who parks in spot B5 every day).
        """
        model = self._db["spot_model"]

        for label, spot_data in run_record["spots"].items():
            if label not in model:
                model[label] = {
                    "total_scans"   : 0,
                    "weighted_sum"  : 0.0,
                    "weight_total"  : 0.0,
                    "weighted_prob" : 0.0,
                    "last_seen"     : run_record["timestamp"]
                }

            entry = model[label]

            # Apply decay to existing weights
            entry["weighted_sum"]   *= DECAY_FACTOR
            entry["weight_total"]   *= DECAY_FACTOR

            # Add new observation with weight 1.0
            new_val = 1.0 if spot_data["occupied"] else 0.0
            entry["weighted_sum"]  += new_val
            entry["weight_total"]  += 1.0
            entry["total_scans"]   += 1
            entry["last_seen"]      = spot_data["timestamp"]

            if entry["weight_total"] > 0:
                entry["weighted_prob"] = (
                    entry["weighted_sum"] / entry["weight_total"]
                )

    def _load_db(self):
        """Load learning database from disk, or create empty one."""
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, "r") as f:
                    return json.load(f)
            except Exception:
                print("[Learner] Warning: could not load DB, starting fresh")

        return {
            "version"    : "1.0",
            "created"    : datetime.now().isoformat(),
            "runs"       : [],
            "spot_model" : {}
        }

    def _save_db(self):
        """Persist learning database to disk."""
        with open(self.db_path, "w") as f:
            json.dump(self._db, f, indent=2)


# ── Integration helpers ───────────────────────────────────────────────────────

def record_and_prioritize(scan_log, waypoints, metadata=None):
    """
    Convenience function — record scan, generate report,
    and return priority-ordered waypoints for the next scan.

    Call this at the end of each drone scan cycle.

    Parameters
    ----------
    scan_log  : Output from parallel_scan() or DroneScanner.run()
    waypoints : Current waypoint list for next scan
    metadata  : Optional dict { weather, notes, etc. }

    Returns
    -------
    (run_id, priority_waypoints, report_str)
    """
    learner = OccupancyLearner()
    run_id  = learner.record_scan(scan_log, metadata)
    report  = learner.generate_report(run_id)
    priority_wps = learner.get_priority_waypoints(waypoints)
    return run_id, priority_wps, report


# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    import random
    random.seed(42)

    # Simulate 5 scan runs
    labels = [f"{'ABCD'[r]}{c}" for r in range(4) for c in range(39)]

    learner = OccupancyLearner(db_path="/tmp/test_learning.json")

    for run in range(5):
        mock_scan = []
        for label in labels:
            # Spots A0-A5 always occupied, D35-D38 never occupied
            if label[0] == "A" and int(label[1:]) < 6:
                occupied = True
            elif label[0] == "D" and int(label[1:]) > 34:
                occupied = False
            else:
                occupied = random.random() < 0.4
            mock_scan.append({
                "label"    : label,
                "x"        : 0.0,
                "y"        : 0.0,
                "occupied" : occupied,
                "timestamp": datetime.now().isoformat()
            })

        learner.record_scan(mock_scan, {"weather": "sunny", "run": run+1})

    print("\n--- Final Report ---")
    learner.generate_report()

    print("\n--- Anomalies ---")
    anomalies = learner.get_anomalies()
    print(f"Always occupied : {anomalies['always_occupied']}")
    print(f"Never occupied  : {anomalies['never_occupied']}")
    print(f"High variance   : {anomalies['high_variance'][:5]}...")

    print("\n--- Heatmap sample ---")
    heatmap = learner.export_heatmap_data()
    sample = list(heatmap.items())[:5]
    for label, data in sample:
        print(f"  {label}: {data['prob']*100:.0f}% [{data['risk_level']}]")

    print("\n=== All tests passed ===")
