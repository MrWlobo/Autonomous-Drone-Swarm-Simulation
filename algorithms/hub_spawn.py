from agents.drop_zone import DropZone
from agents.package import Package
from algorithms.base import Strategy, HubAction, DroneAction
from mesa.discrete_space import Cell
from agents.drone import Drone
from agents.hub import Hub

class HubSpawn(Strategy):
    def register_drone(self, drone):
        pass

    def decide(self, agent):
        if isinstance(agent, Drone):
            return self.decide_for_drone(agent)
        elif isinstance(agent, Hub):
            return self.decide_for_hub(agent)
        elif isinstance(agent, Package):
            return (None, None)
        elif isinstance(agent, DropZone):
            return (None, None)
        return (None, None)
    
        
    def decide_for_drone(self, drone: Drone):
        # if idle, go home to Hub
        if not drone.package and not drone.assigned_packages:
            if drone.hub and drone.hub.cell:
                return self.move_towards(drone, drone.hub.cell)
            return DroneAction.WAIT, None # Safety fallback

        # drop off package
        elif drone.package and drone.cell == drone.package.drop_zone.cell:
            return DroneAction.DROPOFF_PACKAGE, drone.cell
        
        # pick up assigned package
        elif not drone.package and drone.assigned_packages:
            target_package = drone.assigned_packages[0]
            if drone.cell == target_package.cell:
                return DroneAction.PICKUP_PACKAGE, target_package
            else:
                return self.move_towards(drone, target_package.cell)
        
        # carrying package, move to DropZone
        elif drone.package:
            return self.move_towards(drone, drone.package.drop_zone.cell)
        
        return DroneAction.WAIT, None
    
    def decide_for_hub(self, hub: Hub):
        # create requests
        if len(hub.package_requests) + len(hub.active_drones) == 0:
            if hub.model.random.randint(1, 100) <= 5:
                return HubAction.CREATE_DELIVERY_REQUEST, None
            else:
                return HubAction.WAIT, None
        
        # deploy Drones
        elif len(hub.package_requests) != 0:
            return HubAction.DEPLOY_DRONE, None
        
        # collect Drones
        elif len(hub.active_drones) != 0:
            for drone in hub.active_drones:

                if drone.cell is None:
                    continue # skip drones that are already stored/collected
                
                # check if drone is at the hub location
                if drone.cell.coordinate == hub.cell.coordinate:
                    if not drone.assigned_packages and drone.package is None:
                        return HubAction.COLLECT_DRONE, drone

        return HubAction.WAIT, None

    def move_towards(self, drone: Drone, target_cell: Cell):
        if drone.cell == target_cell:
            return DroneAction.WAIT, drone.cell

        neighbors = list(drone.cell.neighborhood)
        
        # find the neighbor with the smallest distance to the target
        best_neighbor = min(
            neighbors, 
            key=lambda n: self.hex_distance(n.coordinate, target_cell.coordinate)
        )
        
        return DroneAction.MOVE_TO_CELL, best_neighbor


    def hex_distance(self, a, b):
        """
        Calculates distance between two cells on an Offset Hex Grid (Odd-R).
        """
        col1, row1 = a
        col2, row2 = b

        # convert offset (odd-r) to axial
        q1 = col1 - (row1 - (row1 & 1)) // 2
        r1 = row1
        
        q2 = col2 - (row2 - (row2 & 1)) // 2
        r2 = row2

        # calculate distance
        dq = q1 - q2
        dr = r1 - r2
        return (abs(dq) + abs(dq + dr) + abs(dr)) / 2

    def grid_init(self, model):
        # safely pick random cells for hubs
        num_cells = model.num_hubs
        
        # get all cells from the grid iterator
        all_cells = list(model.grid) 
        
        if len(all_cells) < num_cells:
            raise ValueError("Grid too small for number of hubs")

        cells = model.random.sample(all_cells, k=num_cells)
        
        Hub.create_agents(
            model=model,
            n=model.num_hubs,
            cell=cells, # create_agents handles the list distribution
        )