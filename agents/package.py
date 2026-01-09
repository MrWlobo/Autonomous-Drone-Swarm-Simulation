from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell

from .drop_zone import DropZone
if TYPE_CHECKING:
    from model.model import DroneModel

class Package(CellAgent):
    """A package to be delivered to a destination."""
    def __init__(self, model: DroneModel, cell: Cell = None, height: float = 0.0, weight: float = 0.0, drop_zone: DropZone = None):
        super().__init__(model)
        self.cell = cell
        self.height = height
        self.weight = weight
        self.drop_zone = drop_zone
        self.model: DroneModel = model
        
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
    
    def deliver(self):
        self.model.agents.remove(self.drop_zone)
        self.model.completed_deliveries.append(self)
    
    