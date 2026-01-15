from __future__ import annotations
from typing import TYPE_CHECKING, Tuple, Union
import random

from algorithms.base import Strategy, DroneAction, HubAction
from agents.drone import Drone
from agents.hub import Hub
from utils.distance import hex_distance, hex_vector_len

if TYPE_CHECKING:
    from model.model import DroneModel
    from mesa.discrete_space import Cell

class Dummy(Strategy):
    """
    A simple baseline strategy with basic flight physics.
    """

    def __init__(self, model: DroneModel) -> None:
        self.model = model
        self.assignments_done = False
    
    def step(self):
        if not self.assignments_done:
            packages = self.model.get_packages()
            drones = self.model.get_drones()
            if drones:
                for i, package in enumerate(packages):
                    drone_index = i % len(drones)
                    drones[drone_index].add_package(package)
                
            self.assignments_done = True

    def register_drone(self, drone: Drone):
        pass

    def decide(self, agent: Union[Drone, Hub]) -> Tuple[Union[DroneAction, HubAction], object]:
        if isinstance(agent, Drone):
            return self._decide_drone(agent)
        elif isinstance(agent, Hub):
            return self._decide_hub(agent)
        else:
            return None, None

    def _decide_drone(self, drone: Drone) -> Tuple[DroneAction, object]:
        if drone.cell is None:
            return DroneAction.WAIT, None
        
        if drone.package:
            target_cell = drone.package.drop_zone.cell
            
            if drone.cell == target_cell:
                if hex_vector_len(drone.cur_speed_vec) <= 1:
                    if drone.get_altitude_above_ground() <= 10:
                        return DroneAction.DROPOFF_PACKAGE, None
                    else:
                        return DroneAction.DESCENT, 9
                else:
                    return DroneAction.WAIT, None 
            
            return self._move_towards(drone, target_cell)

        elif drone.assigned_packages:
            target_package = drone.assigned_packages[0]
            
            if drone.cell == target_package.cell:
                if hex_vector_len(drone.cur_speed_vec) <= 1:
                    if drone.get_altitude_above_ground() <= 10:
                        return DroneAction.PICKUP_PACKAGE, target_package
                    else:
                        return DroneAction.DESCENT, 9
                else:
                    return DroneAction.WAIT, None
            
            return self._move_towards(drone, target_package.cell)

        else:
            target_hub = drone.hub
            
            if not target_hub:
                min_dist = float('inf')
                nearest_hub = None
                for hub in self.model.get_hubs():
                    if hub.cell:
                        dist = hex_distance(drone.cell, hub.cell)
                        if dist < min_dist:
                            min_dist = dist
                            nearest_hub = hub
                target_hub = nearest_hub

            if target_hub and target_hub.cell:
                if drone.cell == target_hub.cell:
                    if drone.get_altitude_above_ground() > 10:
                        return DroneAction.DESCENT, 9
                    return DroneAction.WAIT, None
                
                return self._move_towards(drone, target_hub.cell)

            return DroneAction.WAIT, None

    def _decide_hub(self, hub: Hub) -> Tuple[HubAction, object]:
        for agent in hub.cell.agents:
            if isinstance(agent, Drone):
                if (not agent.package and 
                    not agent.assigned_packages and 
                    hex_vector_len(agent.cur_speed_vec) <= 1 and 
                    agent.get_altitude_above_ground() <= 10):
                    
                    return HubAction.COLLECT_DRONE, agent

        if hub.stored_drones and Hub.package_requests:
            safe_to_launch = True
            for agent in hub.cell.agents:
                if isinstance(agent, Drone) and agent.get_altitude_above_ground() < 20:
                    safe_to_launch = False
                    break
            
            if safe_to_launch:
                return HubAction.DEPLOY_DRONE, None

        return HubAction.WAIT, None

    def _move_towards(self, drone: Drone, target_cell: Cell) -> Tuple[DroneAction, object]:
        if drone.cell == target_cell:
            return DroneAction.WAIT, drone.cell
            
        if drone.get_altitude_above_ground() < 40:
            return DroneAction.ASCENT, 55
            
        return DroneAction.MOVE_TO_CELL, target_cell