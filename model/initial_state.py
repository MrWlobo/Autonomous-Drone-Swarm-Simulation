from __future__ import annotations
import abc
from typing import TYPE_CHECKING

from agents.drone import Drone
from agents.drop_zone import DropZone
from agents.package import Package
from agents.hub import Hub
from agents.obstacle import Obstacle

if TYPE_CHECKING:
    from model.model import DroneModel


class InitialStateSetter(abc.ABC):
    """Abstract class for defining methods of setting the initial state of the model."""
    @abc.abstractmethod
    def set_initial_state(model: DroneModel) -> None:
        pass


class RandomInitialStateSetter(InitialStateSetter):
    def set_initial_state(self, model: DroneModel) -> None:
        all_cells = list(model.grid)
        
        drop_zones = []
        dropzone_cells = model.random.choices(all_cells, k=model.num_packages)
        for cell in dropzone_cells:
            dz = DropZone(model, cell)
            
            drop_zones.append(dz)
            
        packages = []
        package_cells = model.random.choices(all_cells, k=model.num_packages)
        for i, cell in enumerate(package_cells):
            p = Package(model, cell, drop_zones[i])
            
            packages.append(p)

        drones = []
        drone_cells = model.random.choices(all_cells, k=model.num_drones)
        for cell in drone_cells:
            d = Drone(model, cell=cell)
            
            drones.append(d)

        for i, package in enumerate(packages):
            drone_index = i % len(drones)
            drones[drone_index].assigned_packages.append(package)
            
        
        hubs = []
        hub_cells = model.random.choices(all_cells, k=model.num_hubs)
        for cell in hub_cells:
            h = Hub(model, cell=cell)
            
            hubs.append(h)
        
        obstacles = []
        obstacle_cells = model.random.choices(all_cells, k=model.num_obstacles)
        for cell in obstacle_cells:
            h = Obstacle(model, cell=cell)
            
            obstacles.append(h)