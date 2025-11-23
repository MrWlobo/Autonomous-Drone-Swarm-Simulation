from agents.drop_zone import DropZone
from agents.package import Package
from algorithms.base import Strategy, DroneAction
from agents.drone import Drone
from utils.distance import hex_distance

class Dummy(Strategy):
    def register_drone(self, drone: Drone):
        pass

    def decide(self, drone: Drone):
        if not drone.package and not drone.assigned_packages:
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
    
    
    def grid_init(self, model):
        all_cells = list(model.grid)
        
        drop_zones = []
        dropzone_cells = model.random.choices(all_cells, k=model.num_packages)
        for cell in dropzone_cells:
            dz = DropZone(model, cell)
            drop_zones.append(dz)
            
        packages = []
        package_cells = model.random.choices(all_cells, k=model.num_packages)
        for i, cell in enumerate(package_cells):
            p = Package(model, cell)
            p.drop_zone = drop_zones[i]
            packages.append(p)

        drones = []
        drone_cells = model.random.choices(all_cells, k=model.num_drones)
        for cell in drone_cells:
            d = Drone(model, cell=cell)
            
            d.assigned_packages = []
            drones.append(d)

        for i, package in enumerate(packages):
            drone_index = i % len(drones)
            drones[drone_index].assigned_packages.append(package)