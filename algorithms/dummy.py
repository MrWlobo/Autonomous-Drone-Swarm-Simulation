from algorithms.base import Strategy, DroneAction
from agents.drone import Drone
from utils.distance import hex_distance

class Dummy(Strategy):
    def register_drone(self, drone: Drone):
        pass

    def decide(self, drone: Drone):
        if drone.check_for_lack_of_energy() or drone.check_for_collision_with_terrain():
            return DroneAction.DESTROY, None

        elif not drone.package and not drone.assigned_packages:
            return DroneAction.WAIT, drone.cell

        elif drone.package and drone.cell == drone.package.drop_zone.cell:
            return DroneAction.DROPOFF_PACKAGE, drone.cell

        elif drone.package is None and drone.assigned_packages:
            package = drone.assigned_packages[0]
            
            if package.cell is None: 
                return DroneAction.WAIT, drone.cell
            
            target_cell = package.cell
            
            if drone.cell == target_cell:
                return DroneAction.PICKUP_PACKAGE, package

            next_step = self.get_next_hex_step(drone.model, drone.cell, target_cell)
            
            return DroneAction.MOVE_TO_CELL, next_step

        else:
            drop_zone = drone.package.drop_zone
            target_cell = drop_zone.cell
            next_step = self.get_next_hex_step(drone.model, drone.cell, target_cell)
            return DroneAction.MOVE_TO_CELL, next_step


    def get_next_hex_step(self, model, current_cell, target_cell):
        """
        Calculates the best neighbor to move to using Axial Distance logic.
        """
        neighbors = list(current_cell.neighborhood)
        
        best_neighbor = min(
            neighbors, 
            key=lambda n: hex_distance(n, target_cell)
        )
        return best_neighbor