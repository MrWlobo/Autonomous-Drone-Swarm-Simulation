from __future__ import annotations
from typing import TYPE_CHECKING

from agents.drop_zone import DropZone
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

    def _astar(self):
        pass

    def _neighbors(self, cell: Cell) -> list[Cell]:
        qrs = xy_to_qrs(cell.coordinate)
        neighbors_qrs = hex_neighbors_qrs(qrs)

        neighbors_xy = [qrs_to_xy(n) for n in neighbors_qrs]

        # Filter to only existing grid cells
        neighbors = []
        for xy in neighbors_xy:
            col, row = xy
            # Manual bounds check instead of in_bounds
            if 0 <= col < self.model.grid.width and 0 <= row < self.model.grid.height:
                neighbors.append(self.coord_map[col, row])

        return neighbors

    def _cost(self):
        pass

    def _build_coord_map(self):
        self.coord_map = {c.coordinate: c for c in self.model.grid.all_cells}