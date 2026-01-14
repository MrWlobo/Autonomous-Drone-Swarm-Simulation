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
        """
        Create and initialize the adjacency matrix for hubs and packages.

        This method first builds the internal coordinate map, then constructs
        an adjacency matrix representing relationships between hubs and packages.
        The matrix has one row per hub and one column per hub and package combined.

        - Rows correspond to hubs.
        - Columns correspond to hubs followed by packages.
        - All values are initialized to 0.

        The resulting matrix is stored in `self.adjacency_matrix`.

        Returns
        -------
        None
            This method modifies internal state but does not return a value.
        """

        self._build_coord_map()

        hub_count = len(self.model.get_hubs())
        package_count = len(self.model.get_packages())
        adj_mat = [[0 for _ in range(hub_count + package_count)] for _ in range(hub_count)]
        self.adjacency_matrix = adj_mat

    def _direct_path(self, start_cell: Cell, target_cell: Cell) -> list[Cell]:
        """
        Return the shortest path between two cells on the hex grid.

        This method constructs the path directly by stepping from the start cell
        toward the target cell without performing any search or pathfinding.
        Since there are no obstacles, each step reduces the hex distance until
        the target is reached.

        - The path always exists if both cells are within grid bounds.
        - Movement cost is uniform for all steps.
        - The returned path includes both the start and target cells.

        Returns
        -------
        list[Cell]
            An ordered list of cells representing the path from the start cell
            to the target cell.
        """

        start_qrs = xy_to_qrs(start_cell.coordinate)
        target_qrs = xy_to_qrs(target_cell.coordinate)

        path = [start_cell]
        current_qrs = start_qrs

        while current_qrs != target_qrs:
            current_qrs = step_towards(current_qrs, target_qrs)
            xy = qrs_to_xy(current_qrs)
            path.append(self.coord_map[xy])

        return path

    def _distance(self, path: list[Cell], hex_size: int = 2, safe_height: int = 10) -> tuple[int, float, float]:
        """
        Calculate the total traversal distance for a given path.

        This method computes the distance required to traverse a path by combining
        horizontal movement across hex cells with vertical movement due to elevation
        changes. The maximum elevation along the path is used to determine the
        required ascent and descent, with an additional safety margin applied.

        - Horizontal distance is based on the number of cells in the path and the
          configured hex size.
        - Vertical distance includes ascent from the start to the highest elevation
          along the path and descent from that elevation to the target.
        - A safety height margin is added to both ascent and descent calculations.

        Parameters
        ----------
        path : list[Cell]
            An ordered list of cells representing the path from the start cell to
            the target cell.
        hex_size : int, optional
            The horizontal distance represented by a single hex cell.
        safe_height : int, optional
            An additional vertical safety margin applied to ascent and descent.

        Returns
        -------
        tuple[int, float, float]
            A tuple containing:
            - The total traversal distance.
            - The total ascent distance.
            - The total descent distance.

        Notes
        -----
        - Elevation values are obtained using `self.model.get_elevation(cell.coordinate)`.
        - The path is assumed to contain at least one cell.
        """

        max_elevation = 0
        for cell in path:
            cell_elevation = self.model.get_elevation(cell.coordinate)
            if cell_elevation > max_elevation:
                max_elevation = cell_elevation

        start_elevation = self.model.get_elevation(path[0].coordinate)
        target_elevation = self.model.get_elevation(path[-1].coordinate)

        ascent_height = max_elevation + safe_height - start_elevation
        descent_height = max_elevation + safe_height - target_elevation

        return len(path) * hex_size, ascent_height, descent_height

    def _estimated_cost(self, drone: Drone, distance: int, ascent_height: float, descent_height: float, estimated_average_speed: float = 12.0) -> float:
        """
        Estimate the energy cost for a drone to traverse a path.

        This method calculates the estimated total energy consumption required for a drone
        to complete a path by combining the estimated costs of horizontal travel,
        vertical ascent, and vertical descent. Each component is evaluated using
        the drone’s energy model.

        - Horizontal cost is based on the total distance and an estimated average speed.
        - Ascent cost accounts for the energy required to gain altitude.
        - Descent cost accounts for the energy required to lose altitude.

        Parameters
        ----------
        drone : Drone
            The drone for which the energy cost is being estimated.
        distance : int
            The total horizontal distance of the path.
        ascent_height : float
            The total vertical ascent required along the path.
        descent_height : float
            The total vertical descent required along the path.
        estimated_average_speed : float, optional
            The assumed average horizontal speed used for the estimation.

        Returns
        -------
        float
            The estimated total energy cost required to traverse the path.
        """

        cost_horizontal = drone.calculate_energy_drain(estimated_average_speed, 0, distance)
        cost_ascent = drone.calculate_energy_drain(0, ascent_height, 0)
        cost_descent = drone.calculate_energy_drain(0, -descent_height, 0)

        return cost_horizontal + cost_ascent + cost_descent

    def _build_coord_map(self) -> None:
        """
        Build a mapping from coordinates to grid cells.

        This method creates a dictionary that maps each cell's coordinate
        to the corresponding `Cell` object for fast lookup. The resulting
        mapping is stored in `self.coord_map`.

        The coordinate map allows efficient access to cells by their
        (x, y) coordinates without scanning the entire grid.

        Returns
        -------
        None
            Modifies internal state but does not return a value.
        """

        self.coord_map = {c.coordinate: c for c in self.model.grid.all_cells}
