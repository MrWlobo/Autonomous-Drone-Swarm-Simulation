from __future__ import annotations
import numpy as np
from pathlib import Path
from typing import TYPE_CHECKING

from agents.drone import Drone
from agents.drop_zone import DropZone
from agents.hub import Hub
from agents.obstacle import Obstacle
from agents.package import Package
from model.initial_state import InitialStateSetter
from model.presets.utils import get_delivery_locations, load_elevation_grid
from .base import Preset

if TYPE_CHECKING:
    from model.model import DroneModel


class Chongqing38774Preset(Preset):
    # one cell is assumed to be about 2m x 2m
    width: int = 1709 // 2
    height: int = 1075 // 2
    background: Path = Path(__file__).parent.parent.parent / "visualization/assets/Chongqing_38774.png"
    show_gridlines = False
    
    
    def set_model_params(self, model: DroneModel) -> None:
        model.width = self.width
        model.height = self.height
        model.background = self.background
        model.show_gridlines = self.show_gridlines
        model.initial_state_setter = Chongqing38774InitialStateSetter()


class Chongqing38774InitialStateSetter(InitialStateSetter):
    """An InitialStateSetter implementation that places drop zones according to Chongqing AOI 38774."""
    def set_initial_state(self, model: DroneModel) -> None:
        
        # set cell elevations
        elevation_data_path = Path(__file__).parent / "elevation/Chongqing_elevation.json"
        elevation_data = load_elevation_grid(elevation_data_path)
        
        
        for cell in model.grid:
            cell_elevation = elevation_data[cell.coordinate]
            model.set_elevation(cell.coordinate, cell_elevation)
        ######
        
        # place agents
        
        available_cells = set(model.grid)
        
        all_coords = np.array([cell.coordinate for cell in model.grid])
        max_q = np.max(all_coords[:, 0]) 
        max_r = np.max(all_coords[:, 1])
        
        drop_zone_coords = get_delivery_locations("Chongqing", model.num_packages, max_q, max_r)
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