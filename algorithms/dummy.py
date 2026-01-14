from __future__ import annotations
from typing import TYPE_CHECKING, Tuple, Union
import random

from algorithms.base import Strategy, DroneAction, HubAction
from agents.drone import Drone
from agents.hub import Hub
from utils.distance import hex_distance

if TYPE_CHECKING:
    from model.model import DroneModel
    from mesa.discrete_space import Cell

class Dummy(Strategy):
    """
    A simple baseline strategy.
    
    Hub Logic:
    1. Collect any idle drones currently at the hub location.
    2. If there are stored drones and global package requests, deploy a drone.
    3. Randomly generate new delivery requests if the queue is low.
    
    Drone Logic:
    1. If carrying a package -> Move to DropZone -> Dropoff.
    2. If assigned a package -> Move to Package location -> Pickup.
    3. If idle -> Move to the nearest Hub -> Wait to be collected.
    """

    def __init__(self, model: DroneModel) -> None:
        self.model = model
        
        packages = model.get_packages()
        drones = model.get_drones()
        for i, package in enumerate(packages):
            drone_index = i % len(drones)
            drones[drone_index].add_package(package)

    def register_drone(self, drone: Drone):
        """No specific initialization needed for dummy strategy."""
        pass

    def decide(self, agent: Union[Drone, Hub]) -> Tuple[Union[DroneAction, HubAction], object]:
        """Dispatch the decision based on agent type."""
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
                return DroneAction.DROPOFF_PACKAGE, None
            
            return DroneAction.MOVE_TO_CELL, target_cell

        elif drone.assigned_packages:
            target_package = drone.assigned_packages[0]
            
            if drone.cell == target_package.cell:
                return DroneAction.PICKUP_PACKAGE, target_package
            
            return DroneAction.MOVE_TO_CELL, target_package.cell
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
                    return DroneAction.WAIT, None
                
                return DroneAction.MOVE_TO_CELL, target_hub.cell

            return DroneAction.WAIT, None

    def _decide_hub(self, hub: Hub) -> Tuple[HubAction, object]:
        for agent in hub.cell.agents:
            if isinstance(agent, Drone):
                if not agent.package and not agent.assigned_packages:
                    return HubAction.COLLECT_DRONE, agent

        if hub.stored_drones and Hub.package_requests:
            return HubAction.DEPLOY_DRONE, None

        return HubAction.WAIT, None