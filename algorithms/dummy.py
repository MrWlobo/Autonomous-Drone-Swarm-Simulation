from agents.package import Package
from algorithms.base import Strategy, DroneAction


class Dummy(Strategy):
    def register_drone(self, drone):
        pass

    def decide(self, drone):
        package = [a for a in self.model.agents if isinstance(a, Package)][0]
        position = package.cell

        new_x, new_y = drone.cell.coordinate
        pack_x, pack_y = position.coordinate

        if new_x != pack_x:
            new_x += int((pack_x - new_x) / abs(pack_x - new_x))
        elif new_y != pack_y:
            new_y += int((pack_y - new_y) / abs(pack_y - new_y))

        target_cell = next(
            (c for c in drone.model.grid.all_cells.cells if c.coordinate == (new_x, new_y)),
            None
        )
        return DroneAction.MOVE_TO_CELL, target_cell