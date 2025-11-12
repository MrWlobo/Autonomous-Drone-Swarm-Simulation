from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent

if TYPE_CHECKING:
    from model.model import DroneModel

class Obstacle(CellAgent):
    """An obstacle that drone's cannot fly over."""
    def __init__(self, model: DroneModel):
        super().__init__(model)
        