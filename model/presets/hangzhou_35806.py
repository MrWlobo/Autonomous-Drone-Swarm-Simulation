from __future__ import annotations
from typing import TYPE_CHECKING
from pathlib import Path
from noise import pnoise2
import numpy as np

from agents.drone import Drone
from agents.drop_zone import DropZone
from agents.hub import Hub
from agents.obstacle import Obstacle
from agents.package import Package
from model.initial_state import InitialStateSetter
from model.presets.utils import get_delivery_locations

from .base import Preset

if TYPE_CHECKING:
    from model.model import DroneModel


class Hangzhou35806Preset(Preset):
    # one cell is assumed to be about 2m x 2m
    width: int = 985 // 2
    height: int = 1310 // 2
    background: Path = Path(__file__).parent.parent.parent / "visualization/assets/Hangzhou_35806.png"
    show_gridlines = False
    
    
    def set_model_params(self, model: DroneModel) -> None:
        model.width = self.width
        model.height = self.height
        model.background = self.background
        model.show_gridlines = self.show_gridlines
        model.initial_state_setter = Hangzhou35806InitialStateSetter()


class Hangzhou35806InitialStateSetter(InitialStateSetter):
    """An InitialStateSetter implementation that drop zones according to Hangzhou AOI 35806."""
    def set_initial_state(self, model: DroneModel) -> None:
        
        # TEMPORARY set random cell elevations
        
        scale = 0.1 
        octaves = 6 
        persistence = 0.5 
        lacunarity = 2.0  
        
        base_height = 100
        height_variance = 50
        
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
        ######
        
        # place agents
        
        available_cells = set(model.grid)
        
        all_coords = np.array([cell.coordinate for cell in model.grid])
        max_q = np.max(all_coords[:, 0]) 
        max_r = np.max(all_coords[:, 1])
        
        drop_zone_coords = get_delivery_locations("Hangzhou", model.num_packages, max_q, max_r)
        drop_zones = []
        for x, y in drop_zone_coords:
            cell = model.grid[x, y]
            dz = DropZone(model, cell)
            
            available_cells.remove(cell)
            
            drop_zones.append(dz)
        
        available_cells = list(available_cells)
        model.random.shuffle(available_cells)
        
        # for now, agents other than drop zones are placed randomly 
        packages = []
        for i in range(model.num_packages):
            cell = available_cells.pop()
            p = Package(model, cell, 0.5, 2, drop_zones[i % len(drop_zones)])
            
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