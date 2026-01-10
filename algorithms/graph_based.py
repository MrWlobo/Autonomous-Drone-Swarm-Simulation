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

    def _astar(self, start_cell: Cell, target_cell: Cell, heuristic: Callable) -> Optional[list[Cell]]:
        """
        Perform A* pathfinding from a start cell to a target cell.

        This method implements the A* search algorithm on a hex grid. It
        maintains an open priority queue of nodes to explore and a closed
        set of already-explored cells. Each node is represented by an
        `AStarCell` that tracks the cost from the start, estimated cost
        to the target, and a parent link for path reconstruction.

        Parameters
        ----------
        start_cell : Cell
            The starting cell for the pathfinding.
        target_cell : Cell
            The target cell to reach.
        heuristic : Callable[[Cell, Cell], int]
            A function that estimates the cost from a cell to the target.

        Returns
        -------
        Optional[list[Cell]]
            A list of cells representing the path from start to target in
            order, or `None` if no path exists.

        Notes
        -----
        - The path is reconstructed using the parent links stored in each
          `AStarCell`.
        - The method uses a priority queue (`heapq`) ordered by f_score
          (g_score + h_score) to efficiently explore nodes.
        - Neighboring cells blocked by obstacles are ignored (handled by
          `_neighbors` method).
        """

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
        """
        Return all valid neighboring cells for a given cell on the hex grid.

        This method converts the cell's coordinate from offset (x, y) space to
        cube (q, r, s) coordinates to determine its six hexagonal neighbors.
        Neighboring cells are then converted back to (x, y) coordinates and
        filtered to ensure they are within grid bounds and not blocked by
        obstacles.

        A cell is considered a valid neighbor if:
        - It lies within the grid boundaries.
        - It does not contain an `Obstacle` agent.

        Parameters
        ----------
        cell : Cell
            The cell for which neighboring cells are requested.

        Returns
        -------
        list[Cell]
            A list of neighboring cells that are reachable from the given cell.
        """

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
        """
        Reconstruct the path from the start cell to the target cell.

        This method follows the parent links of the given `AStarCell` from the
        target back to the start, collecting the associated `Cell` objects.
        The resulting path is returned in start-to-target order.

        Parameters
        ----------
        target : AStarCell
            The target node reached by the A* search.

        Returns
        -------
        list[Cell]
            An ordered list of cells representing the path from the start
            cell to the target cell.
        """

        current = target
        path = []

        while current.parent:
            path.append(current.cell)
            current = current.parent
        path.append(current.cell)

        return list(reversed(path))

    def _cost(self, path: list[Cell], hex_size: int = 2, safe_height: int = 10) -> int:
        """
        Calculate the traversal cost of a path considering distance and elevation.

        The cost is computed as the sum of:
        1. The horizontal distance, proportional to the number of cells in the path
           multiplied by the size of each hex (`hex_size`).
        2. The vertical ascent needed to reach the maximum elevation along the path
           plus a safety margin (`safe_height`) from the start.
        3. The vertical descent needed to descend from the maximum elevation to
           the target cell plus the safety margin.

        Parameters
        ----------
        path : list[Cell]
            An ordered list of cells representing the path from start to target.
        hex_size : int, optional
            The horizontal distance cost per hex cell (default is 2).
        safe_height : int, optional
            The safety margin added to ascent and descent calculations (default is 10).

        Returns
        -------
        int
            The total cost of traversing the path considering distance and elevation.

        Notes
        -----
        - Elevation values are obtained from `self.model.get_elevation(cell.coordinate)`.
        - The method assumes the path contains at least one cell.
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

        return len(path) * hex_size + ascent_height + descent_height

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

class AStarCell:
    """
    Represents a node in the A* pathfinding algorithm.

    Each `AStarCell` wraps a grid `Cell` and stores the scores used by
    the A* algorithm for pathfinding:

    - `g_score`: Cost from the start node to this node.
    - `h_score`: Heuristic estimate of cost from this node to the target.
    - `f_score`: Sum of `g_score` and `h_score` (used for priority in a queue).

    Attributes
    ----------
    cell : Cell
        The grid cell associated with this A* node.
    g_score : int
        Cost from the start cell to this cell.
    h_score : int
        Estimated cost from this cell to the target.
    parent : Optional[AStarCell]
        The previous node in the path (used to reconstruct the path).
    """

    def __init__(self, cell: Cell, g_score: int, h_score: int, parent: Optional["AStarCell"]):
        """
        Initialize an A* node with scores and a parent link.

        Parameters
        ----------
        cell : Cell
            The grid cell this node represents.
        g_score : int
            Cost from the start cell to this node.
        h_score : int
            Estimated cost from this node to the target.
        parent : Optional[AStarCell]
            The parent node in the path (None if this is the start node).
        """
        self.cell = cell
        self.g_score = g_score
        self.h_score = h_score
        self.parent = parent

    @property
    def f_score(self) -> int:
        """
        Total estimated cost (g_score + h_score) for A* pathfinding.

        Returns
        -------
        int
            Sum of g_score and h_score.
        """
        return self.g_score + self.h_score

    def __lt__(self, other: "AStarCell") -> bool:
        """
        Comparison operator for priority queue sorting.

        Nodes are compared by their f_score, allowing use in a min-heap
        or priority queue for efficient A* pathfinding.

        Parameters
        ----------
        other : AStarCell
            Another node to compare against.

        Returns
        -------
        bool
            True if this node's f_score is less than the other node's f_score.
        """
        return self.f_score < other.f_score