from __future__ import annotations

import heapq
from typing import TYPE_CHECKING, Optional, Callable

from agents.drop_zone import DropZone
from agents.obstacle import Obstacle
from agents.package import Package
from algorithms.base import Strategy, HubAction, DroneAction
from mesa.discrete_space import Cell
from agents.drone import Drone
from agents.hub import Hub
from agents.collision import Collision
from utils.distance import *
from utils.agent_utils import get_closest_available_hub

if TYPE_CHECKING:
    from model.model import DroneModel

# IDEA
# 1. Create a graph with vertices being the hubs and drop zones and
#    edges being the shortest paths between two given points.
#    The edges should consider the terrain shape while calculating
#    the best path.
#
# 2. Calculate the energy cost of following the given path by a given drone (different
#    weights of the cargo will result in varying energy loses).
#
# 3. Use an algorithm (Dijkstra, A* etc.) to find the shortest path from drone's hub to
#    the drop zone.
#
# 4A. If drone is able to make the journey to drop zone and back to the hub without its
#     battery life dropping below a safe_battery_level (a variable chosen by user at the
#     beginning of the simulation) it goes using the shortest path.
#
# 4B. If drone is unable to make it, the algorithm searches for the best available path
#     (one leading through hubs where drone can recharge), which is then assigned to the
#     drone. It then uses the chargers on its way to get back to a comfortable energy level.
#
# 4C. If the package is unreachable (eg. cost of getting to it from the nearest hub and back
#     is > 100% of drone;s battery life) the task is discarded and a new one is assigned.
#
# 5. Drone comes back to the nearest hub that still has packages, gets assigned a new task and
#    recharges enough to perform it.
#
# 6. After all deliveries were made, the drones return to the nearest hub and go into rest mode.
#    When they all do so, the simulation ends.

class GraphBased(Strategy):

    def __init__(self, model: DroneModel):
        super().__init__(model)
        self.coord_map = None
        self.adjacency_matrix = None

    def register_drone(self, drone):
        pass

    def decide(self, agent):
        self._create_adjacency_matrix()
        if isinstance(agent, Drone):
            return self.decide_for_drone(agent)
        elif isinstance(agent, Hub):
            return self.decide_for_hub(agent)
        elif isinstance(agent, Package):
            return (None, None)
        elif isinstance(agent, DropZone):
            return (None, None)
        return (None, None)

    def decide_for_drone(self, drone: Drone):
        pass

    def decide_for_hub(self, hub: Hub):
        pass

    def move_towards(self, drone: Drone, target_cell: Cell):
        if drone.cell == target_cell:
            return DroneAction.WAIT, drone.cell
        return DroneAction.MOVE_TO_CELL, target_cell

    def _create_adjacency_matrix(self) -> None:
        self._build_coord_map()

        hub_count = len(self.model.get_hubs())
        package_count = len(self.model.get_packages())
        adj_mat = [[0 for _ in range(hub_count + package_count)] for _ in range(hub_count)]
        self.adjacency_matrix = adj_mat

    def _astar(self, start_cell: Cell, target_cell: Cell, heuristic: Callable) -> Optional[list[Cell]]:
        start = AStarCell(start_cell, 0, heuristic(start_cell, target_cell),  None)
        open_pq = [(start.g_score + start.h_score, start)]
        heapq.heapify(open_pq)
        closed = {}

        while True:
            if not open_pq:
                return None
            f_score, current = heapq.heappop(open_pq)
            closed[current.cell.coordinate] = current

            if current.cell.coordinate == target_cell.coordinate:
                print(self._build_path(current))
                return self._build_path(current)

            for neighbor_cell in self._neighbors(current.cell):
                if neighbor_cell.coordinate in closed:
                    continue

                neighbor = AStarCell(neighbor_cell, current.g_score + 1, heuristic(neighbor_cell, target_cell), current)
                heapq.heappush(open_pq, (neighbor.g_score + neighbor.h_score, neighbor))

    def _neighbors(self, cell: Cell) -> list[Cell]:
        qrs = xy_to_qrs(cell.coordinate)
        neighbors_qrs = hex_neighbors_qrs(qrs)

        neighbors_xy = [qrs_to_xy(n) for n in neighbors_qrs]

        neighbors = []
        for xy in neighbors_xy:
            col, row = xy
            neighbor_cell = None
            if 0 <= col < self.model.grid.width and 0 <= row < self.model.grid.height:
                neighbor_cell = self.coord_map[col, row]
            if neighbor_cell and all(not isinstance(agent, Obstacle) for agent in neighbor_cell.agents):
                neighbors.append(neighbor_cell)

        return neighbors

    def _build_path(self, target: AStarCell) -> list[Cell]:
        current = target
        path = []

        while current.parent:
            path.append(current.cell)
            current = current.parent
        path.append(current.cell)

        return list(reversed(path))

    def _cost(self):
        pass

    def _build_coord_map(self):
        self.coord_map = {c.coordinate: c for c in self.model.grid.all_cells}

class AStarCell:
    def __init__(self, cell: Cell, g_score: int, h_score: int, parent: Optional["AStarCell"]):
        self.cell = cell
        self.g_score = g_score
        self.h_score = h_score
        self.parent = parent

    @property
    def f_score(self):
        return self.g_score + self.h_score

    def __lt__(self, other):
        return self.f_score < other.f_score