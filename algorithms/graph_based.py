from __future__ import annotations

import heapq
import itertools
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
#     is > 100% of drone's battery life) the task is discarded and a new one is assigned.
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
        self.hub_list = list(self.model.get_hubs())
        self.drop_zone_list = list(self.model.get_drop_zones())

        self.distinct_package_cells = []
        for package in self.model.get_packages():
            if not any(p.cell.coordinate == package.cell.coordinate for p in self.distinct_package_cells):
                self.distinct_package_cells.append(package.cell)

        self.adjacency_matrix = None
        self._create_adjacency_matrix()

        self.drone_paths = None
        self._create_drone_paths()

    def register_drone(self, drone):
        pass

    def decide(self, agent):
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

        The resulting matrix is stored in `self.adjacency_matrix`.

        Returns
        -------
        None
            This method modifies internal state but does not return a value.
        """

        self._build_coord_map()

        hub_count = len(self.hub_list)
        package_count = len(self.drop_zone_list)

        distinct_package_count = len(self.distinct_package_cells)

        adj_mat = [[(0, 0., 0.) for _ in range(hub_count + package_count + distinct_package_count)] for _ in range(hub_count + package_count + distinct_package_count)]

        for i in range(hub_count + package_count):
            for j in range(hub_count + package_count):

                if j < hub_count:
                    first_cell = self.hub_list[j].cell
                elif j < hub_count + package_count:
                    first_cell = self.drop_zone_list[j - hub_count].cell
                else:
                    first_cell = self.distinct_package_cells[j - hub_count - package_count]

                if j < hub_count:
                    other_cell = self.hub_list[j].cell
                elif j < hub_count + package_count:
                    other_cell = self.drop_zone_list[j - hub_count].cell
                else:
                    other_cell = self.distinct_package_cells[j - hub_count - package_count]

                path = self._direct_path(first_cell, other_cell)
                adj_mat[i][j] = self._distance(path)

        self.adjacency_matrix = adj_mat

    def _create_drone_paths(self) -> None:
        """
        Computes and stores the optimal path for each drone that is currently
        carrying a package.
        """
        self.drone_paths = {
            drone: self._find_best_path(drone) for drone in self.model.get_drones() if drone.package is not None
        }

    def _assign_nearest_package(self, drone: Drone) -> None:
        cell, distance = None, None
        for current_cell in self.distinct_package_cells:
            current_distance = hex_distance(drone.cell, current_cell)
            if distance is None or distance > current_distance:
                cell = current_cell
                distance = current_distance

        for package in self.model.get_packages():
            if package.cell.coordinate == cell.coordinate and not package.assigned:
                package.assigned = True
                drone.assigned_packages.append(package)

    def _find_best_path(self, drone: Drone):
        """
        Compute the best energy-feasible path for a drone to deliver its assigned package.

        The path starts at the drone's current location, visits the assigned drop zone,
        and ends at the hub that is most energy-efficient to reach from the drop zone.
        Intermediate hubs may be visited for partial recharges if needed.

        The algorithm uses a Dijkstra-like search over the adjacency matrix of hubs
        and drop zones, where edge weights represent estimated energy costs for the drone
        to traverse between points. Hubs allow partial recharges to ensure the drone's
        battery remains above the safe threshold.

        Parameters
        ----------
        drone : Drone
            The drone for which to compute the delivery path. Must have `drone.package`
            assigned.

        Returns
        -------
        list[tuple[Hub | DropZone, float]]
            An ordered list of tuples representing the nodes in the path and the drone's
            battery level after reaching each node. The list includes:
            - The starting node (implicitly the drone's current location),
            - Any intermediate hubs used for partial recharges,
            - The assigned drop zone,
            - The final hub after completing the delivery.

            If no feasible path exists (battery insufficient to reach drop zone or a hub safely),
            returns an empty list.

        Notes
        -----
        - Only the assigned drop zone for the drone's package will be visited; other
          drop zones are ignored.
        - Partial recharge at hubs only restores enough battery to stay above the safe
          threshold and reach the next hub, not a full recharge.
        - The adjacency matrix used for energy calculations must include both hubs and
          drop zones as nodes.
        """

        if not drone.package:
            return []

        start_cell = drone.cell
        drop_idx = len(self.hub_list) + self.drop_zone_list.index(drone.package.drop_zone)
        safe_battery_level = (self.model.drone_stats.drone_safe_battery_level * self.model.drone_stats.drone_battery) // 100

        start_edges = self._initialize_start_edges(drone, start_cell, drop_idx, safe_battery_level)

        # Initialize priority queue
        heap = []
        counter = itertools.count()
        for node_idx, battery_left, cost_so_far, node in start_edges:
            heapq.heappush(heap, (cost_so_far, next(counter), node_idx, battery_left, [(node, 0)]))

        visited = dict()

        while heap:
            cost_so_far, _, node_idx, battery_left, path_so_far = heapq.heappop(heap)

            key = (node_idx, battery_left)
            if key in visited and visited[key] <= cost_so_far:
                continue
            visited[key] = cost_so_far

            # If reached drop zone, append nearest hub as final node
            if node_idx == drop_idx:
                # Find nearest hub from drop zone
                nearest_hub_idx = min(
                    range(len(self.hub_list)),
                    key=lambda h_idx: self._estimated_cost(drone, *self.adjacency_matrix[drop_idx][h_idx])
                )

                # Append final hub to path
                path_so_far.append((self.hub_list[nearest_hub_idx], 0))
                return path_so_far

            # Explore neighbors (hubs + drop zones)
            for neighbor_idx, (distance, ascent, descent) in enumerate(self.adjacency_matrix[node_idx]):
                if distance == 0:
                    continue

                # Disallow going to any drop zone except the assigned one
                if neighbor_idx >= len(self.hub_list) and neighbor_idx != drop_idx:
                    continue

                energy_cost = self._estimated_cost(drone, distance, ascent, descent)
                next_battery = battery_left - energy_cost

                # Partial recharge at hubs
                if neighbor_idx < len(self.hub_list):
                    # Ensure enough to reach nearest hub safely
                    nearest_hub_energy = min(
                        self._estimated_cost(drone, *self.adjacency_matrix[neighbor_idx][h_idx])
                        for h_idx in range(len(self.hub_list))
                    )
                    next_battery = max(next_battery, safe_battery_level + nearest_hub_energy)

                if next_battery < safe_battery_level:
                    continue  # cannot proceed safely

                next_node = self.hub_list[neighbor_idx] if neighbor_idx < len(self.hub_list) \
                    else self.drop_zone_list[neighbor_idx - len(self.hub_list)]

                heapq.heappush(heap, (cost_so_far + energy_cost,
                                      next(counter),
                                      neighbor_idx,
                                      next_battery,
                                      path_so_far + [(next_node, next_battery)]))

        # No feasible path found
        return []

    def _initialize_start_edges(self, drone: Drone, start_cell: Cell, drop_idx: int, safe_battery_level: int):
        """
        Build the initial set of edges for a drone's pathfinding search.

        This method generates the first “edges” from the drone's current location to
        all hubs and the assigned drop zone. Each edge includes the estimated energy
        cost to reach the target and the battery level the drone would have after
        traveling that edge. Hubs allow for a partial recharge to ensure safe battery
        levels for subsequent travel.

        Parameters
        ----------
        drone : Drone
            The drone for which to initialize edges. Must have `drone.package` assigned.
        start_cell : Cell
            The current location of the drone.
        drop_idx : int
            The index of the assigned drop zone in the adjacency matrix (after all hubs).
        safe_battery_level : int
            Minimum battery the drone must have to proceed safely, used to filter edges
            and for partial recharge calculations.

        Returns
        -------
        list[tuple[int, float, float, Hub | Package]]
            A list of tuples representing possible first moves:
            - Index of the target node in the adjacency matrix (hub or drop zone)
            - Battery level after traveling to that node (including partial recharge if a hub)
            - Energy cost to reach that node
            - The actual target object (`Hub` or `Package`)

        Notes
        -----
        - Only the assigned drop zone for the drone’s package is considered; other drop
          zones are ignored.
        - Partial recharge at hubs restores enough battery to remain above `safe_battery_level`
          and reach the nearest hub safely, but not full battery.
        - Edges that would leave the drone below `safe_battery_level` are excluded.
        - The returned edges serve as the initial nodes for the priority queue in
          `_find_best_path`.
        """

        start_edges = []

        # Edges to hubs
        for hub_idx, hub in enumerate(self.hub_list):
            path = self._direct_path(start_cell, hub.cell)
            distance, ascent, descent = self._distance(path)
            energy_cost = self._estimated_cost(drone, distance, ascent, descent)

            # Battery left after moving to this hub
            battery_left = drone.battery - energy_cost
            if battery_left < safe_battery_level:
                continue

            # Partial recharge: enough to reach nearest hub safely
            nearest_hub_energy = min(
                self._estimated_cost(drone, *self.adjacency_matrix[hub_idx][h_idx])
                for h_idx in range(len(self.hub_list))
            )
            battery_left = max(battery_left, safe_battery_level + nearest_hub_energy)

            start_edges.append((hub_idx, battery_left, energy_cost, hub))

        # Edge directly to drop zone
        drop_cell = self.drop_zone_list[drop_idx - len(self.hub_list)].cell
        path = self._direct_path(start_cell, drop_cell)
        distance, ascent, descent = self._distance(path)
        energy_cost = self._estimated_cost(drone, distance, ascent, descent)
        battery_left = drone.battery - energy_cost
        if battery_left >= safe_battery_level:
            start_edges.append((drop_idx, battery_left, energy_cost, drone.package))

        return start_edges

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
            current_qrs = step_towards(current_qrs, target_qrs, self.model.grid.width, self.model.grid.height)
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
