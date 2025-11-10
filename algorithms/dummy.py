from algorithms.base import Strategy, DroneAction


class Dummy(Strategy):
    def register_drone(self, drone):
        pass

    def decide(self, drone):
        target_cell = drone.random.choice(drone.model.grid.all_cells.cells)
        return DroneAction.MOVE_TO_CELL, target_cell