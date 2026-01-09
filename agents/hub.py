from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell

from agents.package import Package
from algorithms.base import HubAction
from agents.drop_zone import DropZone
from agents.drone import Drone

if TYPE_CHECKING:
    from model.model import DroneModel
    
class Hub(CellAgent):
    """Station for drones: charges them, sends them on missions, and collects them upon return."""
    package_requests: list[Package] = []

    def __init__(self, model: DroneModel, cell: Cell = None, capacity: int = 5):
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

    def step(self):
        action, target = self.model.strategy.decide(self)

        if action == HubAction.DEPLOY_DRONE:
            drone = self.stored_drones.pop()
            drone.cell = self.cell
            drone.assigned_packages=[Hub.package_requests.pop()]

        elif action == HubAction.COLLECT_DRONE:
            self.stored_drones.append(target)
            self.incomming_drones.remove(target)
            target.cell = None
            target.hub = None

        elif action == HubAction.CREATE_DELIVERY_REQUEST:
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

        elif action == HubAction.WAIT:
            pass