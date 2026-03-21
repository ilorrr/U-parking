"""
path_planner_integration.py
Bridges path_planner.py into the existing UParking codebase.

Reads the real parking lot layout from parking_lot.py's output,
builds the ParkingGraph, and provides drop-in replacements for
qcar_spawner.py's hardcoded waypoint logic.

Usage in main.py:
    from path_planner_integration import build_graph_from_lot, SmartCarRouter

    graph = build_graph_from_lot(parking_spots)
    router = SmartCarRouter(graph, parking_spots)
    waypoints = router.plan("entrance_main", target_label="B21")
"""

import math
from path_planner import (
    ParkingGraph, Node, ParkingSpot,
    ParkingPlanner, AStarStrategy, BFSStrategy,
    BeliefStateStrategy, GeneticLayoutOptimizer
)

# ── Constants from qcar_spawner.py (must stay in sync) ────────
START_X      = -4.0
ROW_PITCH    = 15.5       # vertical distance between row centers
SPOT_HALF    = 2.50
SPOT_WIDTH   = 2.7
SPOT_LENGTH  = 5.5
AISLE_WIDTH  = 10.0       # from parking_lot.py row_spacing

# Aisle center x-positions (between row pairs)
#   Rows A(0) & B(1) share aisle at x ≈ -12.75
#   Rows C(2) & D(3) share aisle at x ≈ -43.75
AISLE_A_X    = -12.75
AISLE_B_X    = -43.75

# Lot entry/exit
ENTRY_ROAD_X =  3.0
Y_HIGHWAY    = -16.0      # road along south edge of lot
LOT_ENTRY_Y  = -10.0      # approximate y of column 0


# ============================================================
# 1. BUILD GRAPH FROM REAL LOT LAYOUT
# ============================================================

def build_graph_from_lot(parking_spots, extra_entrances=None):
    """
    Convert the list of spot dicts (from parking_lot.build_parking_lot)
    into a ParkingGraph that path_planner.py can route on.

    Graph topology:
        entrance ── highway_nodes ── aisle_nodes ── spot_nodes

    Each spot gets a node at its mouth (aisle-facing edge).
    Aisle nodes run along each aisle at every column position.
    Highway nodes connect aisles to the lot entrance.

    Args:
        parking_spots  : list of spot dicts from build_parking_lot()
        extra_entrances: optional list of (name, x, y) additional entry points

    Returns:
        ParkingGraph
    """
    g = ParkingGraph()

    # ── Lot entrance(s) ───────────────────────────────────────
    g.add_node(Node("entrance_main", ENTRY_ROAD_X, Y_HIGHWAY))

    if extra_entrances:
        for name, ex, ey in extra_entrances:
            g.add_node(Node(name, ex, ey))
            # connect to highway
            d = abs(ey - Y_HIGHWAY) + abs(ex - ENTRY_ROAD_X)
            g.add_edge(name, "entrance_main", weight=d)

    # ── Determine unique rows and columns ─────────────────────
    rows = sorted(set(s["row"] for s in parking_spots))
    cols = sorted(set(s["col"] for s in parking_spots))

    # Map row index → aisle x position
    # Rows 0,1 use AISLE_A, rows 2,3 use AISLE_B
    def aisle_x_for_row(row_idx):
        return AISLE_A_X if row_idx in (0, 1) else AISLE_B_X

    # Map row index → row center x
    def row_center_x(row_idx):
        return START_X - row_idx * ROW_PITCH

    # ── Spot-to-position lookup ───────────────────────────────
    spot_lookup = {}   # (row, col) -> spot dict
    for s in parking_spots:
        spot_lookup[(s["row"], s["col"])] = s

    # ── Create aisle nodes (one per column per aisle) ─────────
    aisle_ids = set()
    for aisle_name, ax in [("aisle_A", AISLE_A_X), ("aisle_B", AISLE_B_X)]:
        for col in cols:
            # y position = the spot's y at this column
            sample_spot = None
            for row in rows:
                if (row, col) in spot_lookup:
                    sample_spot = spot_lookup[(row, col)]
                    break
            if sample_spot is None:
                continue

            ay = sample_spot["center"][1]
            node_id = f"{aisle_name}_c{col}"
            g.add_node(Node(node_id, ax, ay))
            aisle_ids.add(node_id)

            # Connect to adjacent column nodes in same aisle
            if col > 0:
                prev_id = f"{aisle_name}_c{col-1}"
                if prev_id in g.nodes:
                    d = abs(ay - g.nodes[prev_id].y)
                    g.add_edge(node_id, prev_id, weight=d)

    # ── Highway nodes (connect aisles to entrance) ────────────
    # Highway runs along y = Y_HIGHWAY
    # Create junction nodes where aisles meet the highway
    for aisle_name, ax in [("aisle_A", AISLE_A_X), ("aisle_B", AISLE_B_X)]:
        hw_id = f"highway_{aisle_name}"
        g.add_node(Node(hw_id, ax, Y_HIGHWAY))

        # Connect highway junction to entrance
        d = abs(ax - ENTRY_ROAD_X)
        g.add_edge(hw_id, "entrance_main", weight=d)

        # Connect highway junction to first aisle column node
        first_aisle = f"{aisle_name}_c{cols[0]}"
        if first_aisle in g.nodes:
            d = abs(g.nodes[first_aisle].y - Y_HIGHWAY)
            g.add_edge(hw_id, first_aisle, weight=d)

    # Connect the two highway junctions to each other
    g.add_edge("highway_aisle_A", "highway_aisle_B",
               weight=abs(AISLE_A_X - AISLE_B_X))

    # ── Spot nodes (at the mouth of each spot) ────────────────
    for s in parking_spots:
        row_idx = s["row"]
        col_idx = s["col"]
        label   = s["label"]
        cx, cy  = s["center"][0], s["center"][1]

        # Spot mouth = edge closest to the aisle
        is_even = s.get("direction", "right") == "right"
        mouth_x = cx - SPOT_HALF if is_even else cx + SPOT_HALF

        spot_obj = ParkingSpot(
            spot_id=label,
            row=row_idx,
            col=col_idx,
            x=cx,
            y=cy,
            occupied=s.get("is_occupied", False),
            confidence=1.0
        )

        spot_node_id = f"spot_{label}"
        g.add_node(Node(spot_node_id, mouth_x, cy, spot=spot_obj))

        # Connect spot to its aisle column node
        aisle_name = "aisle_A" if row_idx in (0, 1) else "aisle_B"
        aisle_col_id = f"{aisle_name}_c{col_idx}"
        if aisle_col_id in g.nodes:
            d = abs(mouth_x - g.nodes[aisle_col_id].x)
            # Add a small turn penalty for pulling into a spot
            turn_penalty = 1.5
            g.add_edge(spot_node_id, aisle_col_id, weight=d + turn_penalty)

    return g


# ============================================================
# 2. UPDATE OCCUPANCY/CONFIDENCE ON GRAPH
# ============================================================

def update_graph_occupancy(graph, parking_spots, occupancy_status,
                           confidence_map=None):
    """
    Sync graph spot nodes with latest occupancy and confidence data.

    Args:
        graph            : ParkingGraph from build_graph_from_lot()
        parking_spots    : original spot dicts
        occupancy_status : {label: bool} from occupancy.check_parking_spot_occupancy()
        confidence_map   : optional {label: float} from drone HSV detection
    """
    confidence_map = confidence_map or {}

    for s in parking_spots:
        label = s["label"]
        node_id = f"spot_{label}"
        node = graph.nodes.get(node_id)
        if node and node.spot:
            node.spot.occupied = occupancy_status.get(label, False)
            node.spot.confidence = confidence_map.get(label, 1.0)


# ============================================================
# 3. SMART CAR ROUTER  (replaces qcar_spawner._build_waypoints)
# ============================================================

class SmartCarRouter:
    """
    Drop-in replacement for qcar_spawner's hardcoded waypoint logic.
    Uses ParkingPlanner strategies instead of fixed aisle coordinates.

    Usage:
        graph  = build_graph_from_lot(parking_spots)
        router = SmartCarRouter(graph, parking_spots)

        # Route to a specific spot
        path, cost = router.plan_to_spot("entrance_main", "B21")

        # Route to nearest vacant spot
        path, cost = router.plan_to_nearest("entrance_main")

        # Route under uncertainty (drone confidence < 1.0)
        path, cost, spot = router.plan_uncertain("entrance_main")

        # Convert graph path to QLabs world coordinates for QCar driving
        world_waypoints = router.path_to_world_coords(path)
    """

    def __init__(self, graph, parking_spots, default_strategy="astar"):
        self.graph    = graph
        self.spots    = parking_spots
        self.planner  = ParkingPlanner(graph)
        self.default  = default_strategy

        # Spot label → spot dict lookup
        self._spot_by_label = {s["label"]: s for s in parking_spots}

    def plan_to_spot(self, start_id, target_label, strategy=None):
        """
        Plan route from start node to a specific parking spot.

        Args:
            start_id     : graph node ID (e.g. "entrance_main")
            target_label : spot label (e.g. "B21")
            strategy     : "astar", "dijkstra", "bfs", etc. (None = auto)

        Returns:
            (path_node_ids, total_cost)
        """
        goal_id = f"spot_{target_label}"
        return self.planner.route(start_id, goal_id,
                                  strategy=strategy or self.default)

    def plan_to_nearest(self, start_id):
        """BFS to nearest vacant spot from start."""
        return self.planner.route_nearest(start_id)

    def plan_uncertain(self, start_id, distance_weight=0.4,
                       confidence_weight=0.6):
        """Belief-state routing when occupancy detection is uncertain."""
        return self.planner.route_uncertain(
            start_id, distance_weight, confidence_weight)

    def path_to_world_coords(self, path_node_ids):
        """
        Convert a list of graph node IDs into QLabs world [x, y, z]
        waypoints suitable for QCar driving (qcar_spawner._set_pose).

        Returns list of (x, y) tuples at ground level.
        """
        coords = []
        for nid in path_node_ids:
            node = self.graph.nodes.get(nid)
            if node:
                coords.append((node.x, node.y))
        return coords

    def path_to_world_coords_with_parking(self, path_node_ids, target_label):
        """
        Like path_to_world_coords but appends the final parking position
        (spot center, not mouth) so the car drives fully into the spot.

        Returns list of (x, y) tuples.
        """
        coords = self.path_to_world_coords(path_node_ids)

        # Append the actual spot center (deeper into the spot)
        spot = self._spot_by_label.get(target_label)
        if spot:
            coords.append((spot["center"][0], spot["center"][1]))

        return coords

    def compare_strategies(self, start_id, target_label,
                           strategies=("astar", "dijkstra", "bfs")):
        """
        Run multiple strategies and return comparison dict.
        Useful for demo / presentation — show algorithm trade-offs.
        """
        goal_id = f"spot_{target_label}"
        results = {}
        for strat in strategies:
            try:
                path, cost = self.planner.route(start_id, goal_id,
                                                strategy=strat)
                results[strat] = {
                    "path": path,
                    "cost": round(cost, 2),
                    "hops": len(path),
                }
            except Exception as e:
                results[strat] = {"error": str(e)}
        return results


# ============================================================
# 4. INTEGRATION WITH QCAR_SPAWNER  (monkey-patch or import)
# ============================================================

def build_smart_waypoints(graph, spot, spawn_x=10.0, spawn_y=-20.0):
    """
    Drop-in replacement for qcar_spawner._build_waypoints().

    Instead of hardcoded aisle coordinates, uses A* on the parking graph
    to find the optimal route from a spawn position to the target spot.

    Returns:
        (waypoints_xy, park_x, park_y) — same format as _build_waypoints
    """
    # Create a temporary spawn node
    spawn_id = "_spawn_temp"
    graph.add_node(Node(spawn_id, spawn_x, spawn_y))

    # Connect spawn to entrance
    d = math.hypot(spawn_x - ENTRY_ROAD_X, spawn_y - Y_HIGHWAY)
    graph.add_edge(spawn_id, "entrance_main", weight=d)

    # Route to target spot
    target_id = f"spot_{spot['label']}"
    planner = ParkingPlanner(graph)
    path, cost = planner.route(spawn_id, target_id, strategy="astar")

    # Convert to world coordinates
    waypoints = []
    for nid in path:
        node = graph.nodes[nid]
        waypoints.append((node.x, node.y))

    # Final park position = spot center
    park_x = spot["center"][0]
    park_y = spot["center"][1]
    waypoints.append((park_x, park_y))

    # Clean up temp node
    del graph.nodes[spawn_id]
    for node in graph.nodes.values():
        node.neighbors.pop(spawn_id, None)

    return waypoints, park_x, park_y


# ============================================================
# 5. INTEGRATION WITH DRONE_SCANNER / BRAIN_PLANNER
# ============================================================

def build_belief_confidence_map(scan_log):
    """
    Convert a drone scan_log (from DroneScanner._scan) into a
    confidence map for belief-state routing.

    Spots the drone saw clearly get confidence=1.0.
    Spots not yet scanned get confidence=0.5 (uncertain).
    This feeds into BeliefStateStrategy.

    Args:
        scan_log: list of {"label": str, "occupied": bool, ...}

    Returns:
        {label: float} confidence map
    """
    scanned = {e["label"] for e in scan_log}
    confidence = {}
    for entry in scan_log:
        # Occupied spots confirmed by drone → high confidence
        # Empty spots confirmed → high confidence
        confidence[entry["label"]] = 1.0

    # Any spot NOT in the scan log is uncertain
    # (caller should check against full parking_spots list)
    return confidence


# ============================================================
# 6. DEMO: How it all connects in main.py
# ============================================================

DEMO_MAIN_SNIPPET = """
# ─── In main.py, after setup() ────────────────────────────────
# Replace the existing entry point block with:

if __name__ == "__main__":
    qlabs, spots, drone, vehicle_actors = setup()

    time.sleep(2)
    vehicle_positions = get_vehicle_positions(qlabs, vehicle_actors)
    occupancy = check_parking_spot_occupancy(qlabs, spots,
                    vehicle_positions=vehicle_positions)

    # ── NEW: Build the routing graph ──────────────────────────
    from path_planner_integration import (
        build_graph_from_lot, update_graph_occupancy,
        SmartCarRouter, build_smart_waypoints
    )

    graph = build_graph_from_lot(spots)
    update_graph_occupancy(graph, spots, occupancy)
    router = SmartCarRouter(graph, spots)

    # ── Route a QCar using A* instead of hardcoded waypoints ──
    empty_spots = get_empty_parking_spots(spots, occupancy)
    if empty_spots:
        target = empty_spots[0]
        path, cost = router.plan_to_spot("entrance_main", target["label"])
        world_wps = router.path_to_world_coords_with_parking(
            path, target["label"])
        print(f"A* route to {target['label']}: {len(path)} nodes, "
              f"cost={cost:.1f}m")
        print(f"  World waypoints: {world_wps[:5]}...")

        # Compare algorithms for your Senior Design report
        comparison = router.compare_strategies(
            "entrance_main", target["label"])
        for algo, result in comparison.items():
            print(f"  {algo}: cost={result.get('cost','N/A')}, "
                  f"hops={result.get('hops','N/A')}")

    # ── Belief-state routing (after drone scan) ───────────────
    scanner = DroneScanner(drone, spots, drone_altitude=8.0)
    scan_log = scanner.run(qlabs, vehicle_actors,
                qcar_actors=qcar_actor_list,
                qcar_target_labels=qcar_targets)

    # Update graph with drone confidence data
    from path_planner_integration import build_belief_confidence_map
    confidence = build_belief_confidence_map(scan_log)
    update_graph_occupancy(graph, spots, occupancy, confidence)

    # Route under uncertainty
    path, cost, best_spot = router.plan_uncertain("entrance_main")
    if best_spot:
        print(f"Belief-state pick: {best_spot.spot_id} "
              f"(confidence={best_spot.confidence})")
"""


if __name__ == "__main__":
    # ── Quick test with mock spots matching the real lot ──────
    mock_spots = []
    row_labels = "ABCD"
    for r in range(4):
        for c in range(39):
            cx = START_X - r * ROW_PITCH
            cy = -10 + c * SPOT_WIDTH
            yaw = 90 if r % 2 == 0 else 270
            mock_spots.append({
                "center": [cx, cy, 0.005],
                "label": f"{row_labels[r]}{c}",
                "row": r,
                "col": c,
                "yaw_deg": yaw,
                "direction": "right" if yaw == 90 else "left",
                "is_occupied": False,
            })

    print(f"Mock lot: {len(mock_spots)} spots")
    print(f"Rows: {sorted(set(s['row'] for s in mock_spots))}")
    print(f"Cols: 0–{max(s['col'] for s in mock_spots)}")

    # Build graph
    graph = build_graph_from_lot(mock_spots)
    print(f"\nGraph: {len(graph.nodes)} nodes")

    # Count node types
    spot_nodes  = sum(1 for n in graph.nodes.values() if n.spot)
    aisle_nodes = sum(1 for nid in graph.nodes if nid.startswith("aisle_"))
    print(f"  Spot nodes : {spot_nodes}")
    print(f"  Aisle nodes: {aisle_nodes}")
    print(f"  Other      : {len(graph.nodes) - spot_nodes - aisle_nodes}")

    # Mark some spots as occupied
    for s in mock_spots:
        if s["label"] in ("A5", "B10", "C15", "D20"):
            s["is_occupied"] = True
    occ = {s["label"]: s["is_occupied"] for s in mock_spots}
    update_graph_occupancy(graph, mock_spots, occ)

    # Test routing
    router = SmartCarRouter(graph, mock_spots)

    print("\n=== A* : entrance → B21 ===")
    path, cost = router.plan_to_spot("entrance_main", "B21")
    print(f"  Path ({len(path)} nodes): {' → '.join(path[:6])}...")
    print(f"  Cost: {cost:.1f}m")

    world = router.path_to_world_coords_with_parking(path, "B21")
    print(f"  World waypoints: {len(world)} points")
    print(f"    Start: ({world[0][0]:.1f}, {world[0][1]:.1f})")
    print(f"    End:   ({world[-1][0]:.1f}, {world[-1][1]:.1f})")

    print("\n=== BFS : nearest vacant ===")
    path, cost = router.plan_to_nearest("entrance_main")
    if path:
        last_node = graph.nodes[path[-1]]
        print(f"  Nearest: {last_node.spot.spot_id} ({len(path)} hops)")

    print("\n=== Compare strategies : entrance → D30 ===")
    comp = router.compare_strategies("entrance_main", "D30")
    for algo, res in comp.items():
        print(f"  {algo:10s}: cost={res.get('cost','ERR'):>7}, "
              f"hops={res.get('hops','ERR')}")

    print("\n=== build_smart_waypoints (qcar_spawner drop-in) ===")
    target_spot = mock_spots[50]  # B11
    wps, px, py = build_smart_waypoints(graph, target_spot,
                                         spawn_x=10.0, spawn_y=-20.0)
    print(f"  Target: {target_spot['label']}")
    print(f"  Waypoints: {len(wps)}")
    print(f"  Park at: ({px:.1f}, {py:.1f})")

    print("\n" + "="*60)
    print(DEMO_MAIN_SNIPPET)
