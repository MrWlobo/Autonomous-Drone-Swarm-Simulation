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
    """Abstract class for defining methods of setting the initial state of the model.
    (initial drone positions, hub locations etc.).
    """
    @abc.abstractmethod
    def set_initial_state(model: DroneModel) -> None:
        pass


class RandomInitialStateSetter(InitialStateSetter):
    """An InitialStateSetter implementation that places agents randomly on the grid."""
    def set_initial_state(self, model: DroneModel) -> None:
        available_cells = list(model.grid)
        model.random.shuffle(available_cells)
        
        drop_zones = []
        for _ in range(model.num_packages):
            cell = available_cells.pop()
            dz = DropZone(model, cell)
            
            drop_zones.append(dz)
            
        packages = []
        for i in range(model.num_packages):
            cell = available_cells.pop()
            p = Package(model, cell, drop_zones[i])
            
            packages.append(p)

        drones = []
        for _ in range(model.num_drones):
            cell = available_cells.pop()
            d = Drone(model, cell=cell)
            
            drones.append(d)

        for i, package in enumerate(packages):
            drone_index = i % len(drones)
            drones[drone_index].assigned_packages.append(package)
            
        
        hubs = []
        for _ in range(model.num_hubs):
            cell = available_cells.pop()
            h = Hub(model, cell=cell)
            
            hubs.append(h)
        
        obstacles = []
        for _ in range(model.num_obstacles):
            cell = available_cells.pop()
            h = Obstacle(model, cell=cell)
            
            obstacles.append(h)