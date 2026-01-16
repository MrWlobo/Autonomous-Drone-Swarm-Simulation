from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell

from agents.package import Package
from algorithms.base import HubAction
from agents.drop_zone import DropZone

if TYPE_CHECKING:
    from agents.drone import Drone
    from model.model import DroneModel
    
class Hub(CellAgent):
    """Station for drones: charges them, sends them on missions, and collects them upon return."""
    package_requests: list[Package] = []

    def __init__(self, model: DroneModel, cell: Cell = None, capacity: int = 10):
        super().__init__(model)
        self.stored_drones: list[Drone] = []
        self.incomming_drones: set[Drone] = set()
        self.model: DroneModel = model
        self.cell: Cell = cell
        self.capacity = capacity
        
        if cell:
            cell.add_agent(self)
    
    def __eq__(self, other):
        if other is None:
            return False
        if not isinstance(other, type(self)):
            return False
        return other is not None and self.unique_id == other.unique_id
    
    def __hash__(self):
        return hash(self.unique_id)
    
    def is_full(self):
        """Bool value, includes only stored drones, NOT INCOMING DRONES"""
        return len(self.stored_drones) == self.capacity

    def step(self):
        action, target = self.model.strategy.decide(self)

        if action == HubAction.DEPLOY_DRONE:
            self.deploy_drone(target)

        elif action == HubAction.COLLECT_DRONE:
            self.collect_drone(target)

        elif action == HubAction.CREATE_DELIVERY_REQUEST:
            self.create_delivery_request()

        elif action == HubAction.WAIT:
            pass

    def deploy_drone(self, assign_from_requests: bool = True):
        if assign_from_requests is None:
            assign_from_requests = True
        drone = self.stored_drones.pop()
        drone.cell = self.cell
        drone.altitude = 10 + self.model.get_elevation(self.cell.coordinate)
        if assign_from_requests:
            drone.assigned_packages = [Hub.package_requests.pop()]
        drone.in_hub = False

    def collect_drone(self, target):
        self.stored_drones.append(target)
        self.incomming_drones.discard(target)
        target.cell = None
        target.hub = None
        target.in_hub = True

    def create_delivery_request(self):
        empty_cells = [c for c in self.model.grid.all_cells.cells if len(c.agents) == 0]
        if len(empty_cells) < 2:
            return
        self.model.random.shuffle(empty_cells)
        package = Package(
            model=self.model,
            cell=empty_cells[0],
            drop_zone=DropZone(self.model, empty_cells[1]),
            weight=1,
            height=0.1,
        )
        Hub.package_requests.append(package)