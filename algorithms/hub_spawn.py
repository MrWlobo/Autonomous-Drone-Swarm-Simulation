from agents.drop_zone import DropZone
from agents.package import Package
from algorithms.base import Strategy, HubAction, DroneAction
from mesa.discrete_space import Cell
from agents.drone import Drone
from agents.hub import Hub
from utils.distance import hex_distance

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
        return DroneAction.MOVE_TO_CELL, target_cell