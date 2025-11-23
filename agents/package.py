from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell

from .drop_zone import DropZone
if TYPE_CHECKING:
    from model.model import DroneModel

class Package(CellAgent):
    """A package to be delivered to a destination."""
    def __init__(self, model: DroneModel, cell: Cell = None, drop_zone: DropZone = None):
        super().__init__(model)
        self.cell = cell
        self.drop_zone = drop_zone
        
        if cell:
            cell.add_agent(self)
            self.pos = cell.coordinate
        else:
            self.pos = None

    def __eq__(self, other: Package):

        return other is not None and self.unique_id == other.unique_id
    
    def __hash__(self):
        return hash(self.unique_id)
    