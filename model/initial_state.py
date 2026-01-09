from __future__ import annotations
import abc
from typing import TYPE_CHECKING
from noise import pnoise2

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
        # set random cell elevations
        
        scale = 0.1       # How "zoomed in" the terrain is. Lower = wider hills. Higher = jagged.
        octaves = 6       # Level of detail (more octaves = more "bumpy" texture)
        persistence = 0.5 # How much the smaller details affect the shape
        lacunarity = 2.0  # How much detail is added at each octave
        
        base_height = 100
        height_variance = 50 # Heights will roughly range from base to base + variance
        
        seed_x = model.random.randint(0, 1000)
        seed_y = model.random.randint(0, 1000)

        for cell in model.grid:
            q, r = cell.coordinate
            
            x_coord = (q * scale) + seed_x
            y_coord = (r * scale) + seed_y
            
            noise_val = pnoise2(
                x_coord, 
                y_coord, 
                octaves=octaves, 
                persistence=persistence, 
                lacunarity=lacunarity, 
                repeatx=1024, 
                repeaty=1024, 
                base=0
            )
            
            normalized_val = (noise_val + 1) / 2.0
            
            final_height = int(base_height + (normalized_val * height_variance))
            
            final_height = max(base_height, min(base_height + height_variance, final_height))
            
            model.set_elevation(cell.coordinate, final_height)
        
        # place agents randomly
        
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
            p = Package(model, cell, 0.5, 2, drop_zones[i])
            
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
            o = Obstacle(model, cell=cell)
            obstacles.append(o)

class HubsInitialStateSetter(InitialStateSetter):
    def set_initial_state(self, model: DroneModel) -> None:
        # set random cell elevations
        
        scale = 0.1       # How "zoomed in" the terrain is. Lower = wider hills. Higher = jagged.
        octaves = 6       # Level of detail (more octaves = more "bumpy" texture)
        persistence = 0.5 # How much the smaller details affect the shape
        lacunarity = 2.0  # How much detail is added at each octave
        
        base_height = 100
        height_variance = 50 # Heights will roughly range from base to base + variance
        
        seed_x = model.random.randint(0, 1000)
        seed_y = model.random.randint(0, 1000)

        for cell in model.grid:
            q, r = cell.coordinate
            
            x_coord = (q * scale) + seed_x
            y_coord = (r * scale) + seed_y
            
            noise_val = pnoise2(
                x_coord, 
                y_coord, 
                octaves=octaves, 
                persistence=persistence, 
                lacunarity=lacunarity, 
                repeatx=1024, 
                repeaty=1024, 
                base=0
            )
            
            normalized_val = (noise_val + 1) / 2.0
            
            final_height = int(base_height + (normalized_val * height_variance))
            
            final_height = max(base_height, min(base_height + height_variance, final_height))
            
            model.set_elevation(cell.coordinate, final_height)
        
        # place agents randomly
        
        available_cells = list(model.grid)
        model.random.shuffle(available_cells)

        drones = []
        for _ in range(model.num_drones):
            cell = available_cells.pop()
            d = Drone(model, cell=cell)
            drones.append(d)
            
        hubs = []
        for _ in range(model.num_hubs):
            cell = available_cells.pop()
            h = Hub(model, cell=cell)
            hubs.append(h)
        
        obstacles = []
        for _ in range(model.num_obstacles):
            cell = available_cells.pop()
            o = Obstacle(model, cell=cell)
            obstacles.append(o)

def get_initial_state_setter_instance(name: str) -> InitialStateSetter:
    """Returns an InitialStateSetter subclass instance from a string representing its name.

    Args:
        name (str): Namne of the InitialStateSetter to use.

    Returns:
        InitialStateSetter: An InitialStateSetter subclass instance.
    """
    if name == "random":
        return RandomInitialStateSetter()
    elif name == "hubs":
        return HubsInitialStateSetter()
    else:
        return None