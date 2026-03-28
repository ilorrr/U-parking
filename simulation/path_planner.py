"""
path_planner.py — Strategy-pattern path planner for UParking
Swappable algorithms: A*, Dijkstra's, BFS, Hill-Climbing, Genetic, Simulated Annealing
Designed for QLabs QCar + QDrone2 navigation across 156-spot parking lot.
"""

import heapq
import random
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class ParkingSpot:
    spot_id: str           # e.g. "A-3" (row-column)
    row: int
    col: int
    x: float               # world x coordinate in QLabs
    y: float               # world y coordinate in QLabs
    occupied: bool = False
    confidence: float = 1.0  # 0.0–1.0, used by belief-state search


@dataclass
class Node:
    """Graph node representing a navigable point (aisle intersection, spot entrance, lot entrance)."""
    node_id: str
    x: float
    y: float
    neighbors: dict = field(default_factory=dict)  # {neighbor_id: edge_weight}
    spot: Optional[ParkingSpot] = None              # non-None if this node is a spot entrance

    def __hash__(self):
        return hash(self.node_id)

    def __eq__(self, other):
        return self.node_id == other.node_id


class ParkingGraph:
    """
    Graph representation of the parking lot.
    Nodes = aisle intersections + spot entrances + lot entrance(s).
    Edges = driveable aisle segments with weights (distance, traffic, turn penalty).
    """

    def __init__(self):
        self.nodes: dict[str, Node] = {}

    def add_node(self, node: Node):
        self.nodes[node.node_id] = node

    def add_edge(self, id_a: str, id_b: str, weight: float = 1.0, bidirectional: bool = True):
        self.nodes[id_a].neighbors[id_b] = weight
        if bidirectional:
            self.nodes[id_b].neighbors[id_a] = weight

    def get_vacant_spot_nodes(self, min_confidence: float = 0.5) -> list[Node]:
        """Return nodes linked to vacant spots (optionally filtered by detection confidence)."""
        return [
            n for n in self.nodes.values()
            if n.spot and not n.spot.occupied and n.spot.confidence >= min_confidence
        ]

    def euclidean(self, id_a: str, id_b: str) -> float:
        a, b = self.nodes[id_a], self.nodes[id_b]
        return math.hypot(a.x - b.x, a.y - b.y)

    def manhattan(self, id_a: str, id_b: str) -> float:
        a, b = self.nodes[id_a], self.nodes[id_b]
        return abs(a.x - b.x) + abs(a.y - b.y)


# ---------------------------------------------------------------------------
# Strategy interface
# ---------------------------------------------------------------------------

class PathStrategy(ABC):
    """Base class — all planners implement this."""

    @abstractmethod
    def find_path(
        self,
        graph: ParkingGraph,
        start_id: str,
        goal_id: str,
    ) -> tuple[list[str], float]:
        """
        Returns (path_as_node_ids, total_cost).
        Empty list + inf cost if no path found.
        """
        ...


# ---------------------------------------------------------------------------
# 1. A* Search  (Informed — primary QCar / Drone router)
# ---------------------------------------------------------------------------

class AStarStrategy(PathStrategy):
    """
    A* with pluggable heuristic (default: euclidean).
    Use for: finding the most efficient route from lot entrance to a specific vacant stall.
    """

    def __init__(self, heuristic: str = "euclidean"):
        self.heuristic = heuristic

    def find_path(self, graph: ParkingGraph, start_id: str, goal_id: str):
        h_fn = graph.euclidean if self.heuristic == "euclidean" else graph.manhattan

        open_set: list[tuple[float, str]] = [(0.0, start_id)]
        g_score = {start_id: 0.0}
        came_from: dict[str, str] = {}

        while open_set:
            f_current, current = heapq.heappop(open_set)

            if current == goal_id:
                return self._reconstruct(came_from, current), g_score[current]

            node = graph.nodes[current]
            for nbr_id, weight in node.neighbors.items():
                tentative_g = g_score[current] + weight
                if tentative_g < g_score.get(nbr_id, float("inf")):
                    came_from[nbr_id] = current
                    g_score[nbr_id] = tentative_g
                    f = tentative_g + h_fn(nbr_id, goal_id)
                    heapq.heappush(open_set, (f, nbr_id))

        return [], float("inf")

    @staticmethod
    def _reconstruct(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))


# ---------------------------------------------------------------------------
# 2. Dijkstra's  (Uniform-Cost — cheapest path when turns/traffic weighted)
# ---------------------------------------------------------------------------

class DijkstraStrategy(PathStrategy):
    """
    Dijkstra's = A* with h(n)=0.
    Use for: cheapest path when edges carry heterogeneous costs (traffic, tight turns).
    """

    def find_path(self, graph: ParkingGraph, start_id: str, goal_id: str):
        open_set: list[tuple[float, str]] = [(0.0, start_id)]
        g_score = {start_id: 0.0}
        came_from: dict[str, str] = {}

        while open_set:
            cost, current = heapq.heappop(open_set)

            if current == goal_id:
                return self._reconstruct(came_from, current), cost

            if cost > g_score.get(current, float("inf")):
                continue

            for nbr_id, weight in graph.nodes[current].neighbors.items():
                new_cost = cost + weight
                if new_cost < g_score.get(nbr_id, float("inf")):
                    g_score[nbr_id] = new_cost
                    came_from[nbr_id] = current
                    heapq.heappush(open_set, (new_cost, nbr_id))

        return [], float("inf")

    @staticmethod
    def _reconstruct(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))


# ---------------------------------------------------------------------------
# 3. BFS  (Uninformed — nearest spot by hop count)
# ---------------------------------------------------------------------------

class BFSStrategy(PathStrategy):
    """
    Breadth-First Search — treat every edge as cost 1.
    Use for: finding the nearest available spot when all spots are equal value.
    """

    def find_path(self, graph: ParkingGraph, start_id: str, goal_id: str):
        from collections import deque

        queue = deque([(start_id, [start_id])])
        visited = {start_id}

        while queue:
            current, path = queue.popleft()
            if current == goal_id:
                return path, float(len(path) - 1)

            for nbr_id in graph.nodes[current].neighbors:
                if nbr_id not in visited:
                    visited.add(nbr_id)
                    queue.append((nbr_id, path + [nbr_id]))

        return [], float("inf")

    def find_nearest_vacant(self, graph: ParkingGraph, start_id: str) -> tuple[list[str], float]:
        """BFS bonus: find nearest vacant spot without a specific goal."""
        from collections import deque

        queue = deque([(start_id, [start_id])])
        visited = {start_id}

        while queue:
            current, path = queue.popleft()
            node = graph.nodes[current]
            if node.spot and not node.spot.occupied:
                return path, float(len(path) - 1)

            for nbr_id in node.neighbors:
                if nbr_id not in visited:
                    visited.add(nbr_id)
                    queue.append((nbr_id, path + [nbr_id]))

        return [], float("inf")


# ---------------------------------------------------------------------------
# 4. Hill-Climbing  (Local Search — QCar fine-alignment into spot)
# ---------------------------------------------------------------------------

class HillClimbingStrategy(PathStrategy):
    """
    Greedy local search — always move to the neighbor closest to goal.
    Use for: real-time optimization of car angle/position within a parking spot.
    NOTE: Can get stuck in local minima. Use simulated annealing as fallback.
    """

    def find_path(self, graph: ParkingGraph, start_id: str, goal_id: str):
        current = start_id
        path = [current]
        total_cost = 0.0

        while current != goal_id:
            node = graph.nodes[current]
            if not node.neighbors:
                return [], float("inf")  # dead end

            # Pick neighbor with smallest heuristic to goal
            best_nbr = min(
                node.neighbors.keys(),
                key=lambda n: graph.euclidean(n, goal_id),
            )

            # Stuck — no improvement
            if graph.euclidean(best_nbr, goal_id) >= graph.euclidean(current, goal_id):
                break

            total_cost += node.neighbors[best_nbr]
            current = best_nbr
            path.append(current)

        return path, total_cost


# ---------------------------------------------------------------------------
# 5. Simulated Annealing  (Local Search — escape local maxima)
# ---------------------------------------------------------------------------

class SimulatedAnnealingStrategy(PathStrategy):
    """
    SA with temperature schedule — probabilistically accepts worse moves.
    Use for: escaping local minima when hill-climbing gets stuck
    (e.g., car stuck at an angle where it can't simply pull forward).
    """

    def __init__(self, initial_temp: float = 100.0, cooling_rate: float = 0.995, min_temp: float = 0.01):
        self.initial_temp = initial_temp
        self.cooling_rate = cooling_rate
        self.min_temp = min_temp

    def find_path(self, graph: ParkingGraph, start_id: str, goal_id: str):
        current = start_id
        path = [current]
        total_cost = 0.0
        temp = self.initial_temp

        best_path = list(path)
        best_dist = graph.euclidean(current, goal_id)

        while current != goal_id and temp > self.min_temp:
            node = graph.nodes[current]
            if not node.neighbors:
                break

            # Pick a random neighbor
            nbr_id = random.choice(list(node.neighbors.keys()))
            nbr_dist = graph.euclidean(nbr_id, goal_id)
            cur_dist = graph.euclidean(current, goal_id)
            delta = nbr_dist - cur_dist  # positive = worse

            # Accept if better, or probabilistically if worse
            if delta < 0 or random.random() < math.exp(-delta / temp):
                total_cost += node.neighbors[nbr_id]
                current = nbr_id
                path.append(current)

                if nbr_dist < best_dist:
                    best_dist = nbr_dist
                    best_path = list(path)

            temp *= self.cooling_rate

        return best_path, total_cost


# ---------------------------------------------------------------------------
# 6. Belief-State Search  (Uncertainty — sensor ambiguity)
# ---------------------------------------------------------------------------

class BeliefStateStrategy(PathStrategy):
    """
    Routes toward spots with highest vacancy probability when detection is uncertain.
    Use for: navigating when drone camera / HSV detection confidence is low.
    Combines A* routing with expected-value scoring of candidate spots.
    """

    def __init__(self, heuristic: str = "euclidean"):
        self.a_star = AStarStrategy(heuristic=heuristic)

    def find_best_uncertain_spot(
        self,
        graph: ParkingGraph,
        start_id: str,
        distance_weight: float = 0.4,
        confidence_weight: float = 0.6,
    ) -> tuple[list[str], float, Optional[ParkingSpot]]:
        """
        Score each candidate spot by:
            score = confidence_weight * confidence - distance_weight * normalized_distance
        Then A* route to the best one.
        """
        candidates = graph.get_vacant_spot_nodes(min_confidence=0.0)
        if not candidates:
            return [], float("inf"), None

        # Compute distances to each candidate
        scored = []
        for node in candidates:
            dist = graph.euclidean(start_id, node.node_id)
            scored.append((node, dist))

        max_dist = max(d for _, d in scored) or 1.0

        best_score = -float("inf")
        best_node = None
        for node, dist in scored:
            norm_dist = dist / max_dist
            score = confidence_weight * node.spot.confidence - distance_weight * norm_dist
            if score > best_score:
                best_score = score
                best_node = node

        path, cost = self.a_star.find_path(graph, start_id, best_node.node_id)
        return path, cost, best_node.spot

    def find_path(self, graph: ParkingGraph, start_id: str, goal_id: str):
        """Standard A* fallback for direct routing."""
        return self.a_star.find_path(graph, start_id, goal_id)


# ---------------------------------------------------------------------------
# 7. Genetic Algorithm  (Optimization — lot layout)
# ---------------------------------------------------------------------------

class GeneticLayoutOptimizer:
    """
    Offline optimizer for parking lot layout.
    Chromosome = sequence of spot assignments; fitness = avg distance from entrance + flow score.
    Use for: optimizing the overall flow or designing the best layout for stalls.
    NOT a PathStrategy — runs offline.
    """

    def __init__(
        self,
        population_size: int = 50,
        generations: int = 200,
        mutation_rate: float = 0.05,
        crossover_rate: float = 0.7,
    ):
        self.pop_size = population_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate

    def optimize(
        self,
        spot_positions: list[tuple[float, float]],
        entrance_pos: tuple[float, float],
        fitness_fn=None,
    ) -> list[int]:
        """
        Returns an optimized ordering/assignment of spots.
        Default fitness: minimizes average distance from entrance.
        """
        n = len(spot_positions)

        if fitness_fn is None:
            ex, ey = entrance_pos
            def fitness_fn(chromosome):
                total = sum(
                    math.hypot(spot_positions[i][0] - ex, spot_positions[i][1] - ey) * (rank + 1)
                    for rank, i in enumerate(chromosome)
                )
                return -total  # negative because we maximize fitness

        # Initialize population
        population = [random.sample(range(n), n) for _ in range(self.pop_size)]

        for gen in range(self.generations):
            scored = [(fitness_fn(chrom), chrom) for chrom in population]
            scored.sort(key=lambda x: x[0], reverse=True)

            # Elitism — keep top 10%
            elite_count = max(2, self.pop_size // 10)
            new_pop = [chrom for _, chrom in scored[:elite_count]]

            # Fill rest with crossover + mutation
            while len(new_pop) < self.pop_size:
                p1 = self._tournament_select(scored)
                p2 = self._tournament_select(scored)

                if random.random() < self.crossover_rate:
                    child = self._order_crossover(p1, p2)
                else:
                    child = list(p1)

                if random.random() < self.mutation_rate:
                    self._swap_mutate(child)

                new_pop.append(child)

            population = new_pop

        best = max(population, key=fitness_fn)
        return best

    @staticmethod
    def _tournament_select(scored, k=3):
        tournament = random.sample(scored, min(k, len(scored)))
        return max(tournament, key=lambda x: x[0])[1]

    @staticmethod
    def _order_crossover(p1, p2):
        n = len(p1)
        start, end = sorted(random.sample(range(n), 2))
        child = [-1] * n
        child[start:end] = p1[start:end]
        fill = [g for g in p2 if g not in child[start:end]]
        idx = 0
        for i in range(n):
            if child[i] == -1:
                child[i] = fill[idx]
                idx += 1
        return child

    @staticmethod
    def _swap_mutate(chrom):
        i, j = random.sample(range(len(chrom)), 2)
        chrom[i], chrom[j] = chrom[j], chrom[i]


# ---------------------------------------------------------------------------
# 8. LRTA*  (Online Search — learns over repeated visits)
# ---------------------------------------------------------------------------

class LRTAStarStrategy(PathStrategy):
    """
    Learning Real-Time A* — updates heuristic table after each move.
    Use for: improving parking performance over time by "learning"
    the specific layout of a frequent garage.
    Persists h_table across calls.
    """

    def __init__(self):
        self.h_table: dict[str, float] = {}  # learned heuristic values

    def find_path(self, graph: ParkingGraph, start_id: str, goal_id: str):
        current = start_id
        path = [current]
        total_cost = 0.0
        max_steps = len(graph.nodes) * 3  # safety bound

        for _ in range(max_steps):
            if current == goal_id:
                return path, total_cost

            node = graph.nodes[current]
            if not node.neighbors:
                break

            # For each neighbor, compute f = edge_cost + h(neighbor)
            def f_value(nbr_id):
                edge_cost = node.neighbors[nbr_id]
                h = self.h_table.get(nbr_id, graph.euclidean(nbr_id, goal_id))
                return edge_cost + h

            best_nbr = min(node.neighbors.keys(), key=f_value)

            # Update h(current) = best f among neighbors (learning step)
            self.h_table[current] = f_value(best_nbr)

            total_cost += node.neighbors[best_nbr]
            current = best_nbr
            path.append(current)

        return path, total_cost


# ---------------------------------------------------------------------------
# Context-aware planner (ties it all together)
# ---------------------------------------------------------------------------

class ParkingPlanner:
    """
    Main planner — selects strategy based on context, or lets you override.
    Drop this into your existing QCar / drone scanner modules.

    Usage:
        planner = ParkingPlanner(graph)

        # Auto-select based on context
        path, cost = planner.route(start="entrance_1", goal="A-3")

        # Force a specific algorithm
        path, cost = planner.route(start="entrance_1", goal="A-3", strategy="bfs")

        # Belief-state when detection is uncertain
        path, cost, spot = planner.route_uncertain(start="entrance_1")
    """

    STRATEGIES = {
        "astar":    AStarStrategy,
        "dijkstra": DijkstraStrategy,
        "bfs":      BFSStrategy,
        "hillclimb": HillClimbingStrategy,
        "sa":       SimulatedAnnealingStrategy,
        "lrta":     LRTAStarStrategy,
    }

    def __init__(self, graph: ParkingGraph):
        self.graph = graph
        self._lrta = LRTAStarStrategy()  # persistent learner
        self._belief = BeliefStateStrategy()

    def route(
        self,
        start: str,
        goal: str,
        strategy: Optional[str] = None,
    ) -> tuple[list[str], float]:
        """
        Route from start to goal.
        If strategy is None, auto-selects:
          - Weighted edges → A*
          - Uniform edges → BFS
        """
        if strategy:
            strat_cls = self.STRATEGIES[strategy]
            strat = self._lrta if strategy == "lrta" else strat_cls()
            return strat.find_path(self.graph, start, goal)

        # Auto-select
        if self._has_uniform_weights():
            return BFSStrategy().find_path(self.graph, start, goal)
        return AStarStrategy().find_path(self.graph, start, goal)

    def route_nearest(self, start: str) -> tuple[list[str], float]:
        """BFS to nearest vacant spot — no specific goal needed."""
        return BFSStrategy().find_nearest_vacant(self.graph, start)

    def route_uncertain(
        self,
        start: str,
        distance_weight: float = 0.4,
        confidence_weight: float = 0.6,
    ) -> tuple[list[str], float, Optional[ParkingSpot]]:
        """Belief-state routing when occupancy detection has low confidence."""
        return self._belief.find_best_uncertain_spot(
            self.graph, start, distance_weight, confidence_weight
        )

    def _has_uniform_weights(self) -> bool:
        weights = set()
        for node in self.graph.nodes.values():
            weights.update(node.neighbors.values())
        return len(weights) <= 1


# ---------------------------------------------------------------------------
# Quick demo / test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Build a tiny test lot: entrance → 3 aisle nodes → 4 spots
    g = ParkingGraph()

    g.add_node(Node("entrance", 0.0, 0.0))
    g.add_node(Node("aisle_1", 5.0, 0.0))
    g.add_node(Node("aisle_2", 10.0, 0.0))
    g.add_node(Node("aisle_3", 10.0, 5.0))

    g.add_node(Node("spot_A1", 5.0, 3.0, spot=ParkingSpot("A-1", 0, 0, 5.0, 3.0, occupied=False)))
    g.add_node(Node("spot_A2", 5.0, -3.0, spot=ParkingSpot("A-2", 0, 1, 5.0, -3.0, occupied=True)))
    g.add_node(Node("spot_B1", 10.0, 3.0, spot=ParkingSpot("B-1", 1, 0, 10.0, 3.0, occupied=False, confidence=0.6)))
    g.add_node(Node("spot_B2", 10.0, 8.0, spot=ParkingSpot("B-2", 1, 1, 10.0, 8.0, occupied=False, confidence=0.9)))

    g.add_edge("entrance", "aisle_1", 5.0)
    g.add_edge("aisle_1", "aisle_2", 5.0)
    g.add_edge("aisle_2", "aisle_3", 5.0)
    g.add_edge("aisle_1", "spot_A1", 3.0)
    g.add_edge("aisle_1", "spot_A2", 3.0)
    g.add_edge("aisle_2", "spot_B1", 3.0)
    g.add_edge("aisle_3", "spot_B2", 3.0)

    planner = ParkingPlanner(g)

    print("=== A* : entrance → spot_B2 ===")
    path, cost = planner.route("entrance", "spot_B2", strategy="astar")
    print(f"  Path: {' → '.join(path)}")
    print(f"  Cost: {cost}")

    print("\n=== BFS : nearest vacant from entrance ===")
    path, cost = planner.route_nearest("entrance")
    print(f"  Path: {' → '.join(path)}")
    print(f"  Hops: {cost}")

    print("\n=== Dijkstra : entrance → spot_B1 ===")
    path, cost = planner.route("entrance", "spot_B1", strategy="dijkstra")
    print(f"  Path: {' → '.join(path)}")
    print(f"  Cost: {cost}")

    print("\n=== Belief-State : best spot under uncertainty ===")
    path, cost, spot = planner.route_uncertain("entrance")
    print(f"  Path: {' → '.join(path)}")
    print(f"  Cost: {cost}")
    print(f"  Chosen spot: {spot.spot_id} (confidence={spot.confidence})")

    print("\n=== LRTA* : entrance → spot_B2 (learns over runs) ===")
    for run in range(3):
        path, cost = planner.route("entrance", "spot_B2", strategy="lrta")
        print(f"  Run {run+1}: {' → '.join(path)} | cost={cost}")
