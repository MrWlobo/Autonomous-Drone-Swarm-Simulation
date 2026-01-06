from agents.drop_zone import DropZone
from agents.package import Package
from algorithms.base import Strategy, HubAction, DroneAction
from mesa.discrete_space import Cell
from agents.drone import Drone
from agents.hub import Hub
from agents.collision import Collision
from utils.distance import *
from utils.agent_utils import get_closest_available_hub

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
        elif isinstance(agent, Collision):
            return (None, None)
        
        return (None, None)
    
        
    def decide_for_drone(self, drone: Drone):
        # if idle, go home to Hub
        if drone.cell is not None and not drone.package and not drone.assigned_packages:
            if drone.hub is None:
                drone.hub = get_closest_available_hub(drone.cell, drone.model.get_hubs())
            if drone.hub is not None:
                drone.hub.incomming_drones.add(drone)
                return self.move_towards(drone, drone.hub.cell)

        # drop off package
        elif drone.package and drone.cell == drone.package.drop_zone.cell and hex_vector_len(drone.cur_speed_vec) <= 1:
            return DroneAction.DROPOFF_PACKAGE, drone.cell
        
        # pick up assigned package
        elif not drone.package and drone.assigned_packages:
            target_package = drone.assigned_packages[0]
            if drone.cell == target_package.cell and hex_vector_len(drone.cur_speed_vec) <= 1:
                return DroneAction.PICKUP_PACKAGE, target_package
            else:
                return self.move_towards(drone, target_package.cell)
        
        # carrying package, move to DropZone
        elif drone.package:
            return self.move_towards(drone, drone.package.drop_zone.cell)
        
        return DroneAction.WAIT, None
    
    def decide_for_hub(self, hub: Hub):
        # create requests
        if hub.model.random.randint(1, 100) <= 4:
            return HubAction.CREATE_DELIVERY_REQUEST, None
        
        # deploy Drones
        elif hub.package_requests and hub.stored_drones:
            safe = True
            for drone in hub.model.get_drones():
                if drone.cell is None:
                    continue
                if hex_distance(hub.cell, drone.cell) <= drone.get_acceleration()*2 + 5:
                    safe = False
            if safe:
                print('hub deploy')
                return HubAction.DEPLOY_DRONE, None
        
        # collect Drones
        for drone in hub.model.get_drones():
            if drone.cell is None:
                continue # skip drones that are already stored/collected
            
            # check if drone is at the hub location
            if drone.cell.coordinate == hub.cell.coordinate and drone.cell:
                if len(drone.assigned_packages)==0 and drone.package is None and hex_vector_len(drone.cur_speed_vec) <= 1:
                    if hub.capacity > len(hub.stored_drones):
                        print('hub collect')
                        return HubAction.COLLECT_DRONE, drone

        return HubAction.WAIT, None

    def move_towards(self, drone: Drone, target_cell: Cell):
        if drone.cell == target_cell:
            return DroneAction.WAIT, drone.cell
        return DroneAction.MOVE_TO_CELL, target_cell