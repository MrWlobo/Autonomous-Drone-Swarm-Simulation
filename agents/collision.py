from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell

if TYPE_CHECKING:
    from model.model import DroneModel

class Collision(CellAgent):
    """Result of drone collision that can be visualized."""
    def __init__(self, model: DroneModel, cell: Cell, life_ticks: int = 5):
        super().__init__(model)
        self.life_ticks = life_ticks
        self.model = model
        self.cell = cell
        if cell:
            cell.add_agent(self)


    def step(self) -> None:
        self.life_ticks -= 1
        if self.life_ticks <= 0:
            self.destroy()

    def destroy(self) -> None:
        self.model.agents.remove(self)