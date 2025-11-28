from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell
import logging

from agents.package import Package
from algorithms.base import DroneAction
from utils.distance import hex_distance, hex_vector, hex_vector_len, normalize_hex_vector, xy_to_qrs, qer_to_xy

if TYPE_CHECKING:
    from model.model import DroneModel
    from agents.hub import Hub

class Drone(CellAgent):
    def __init__(self, model: DroneModel, cell: Cell = None, assigned_packages: list[Package] = None, hub: Hub = None):
        super().__init__(model)
        self.speed = model.drone_stats.drone_speed
        self.cur_speed_vec = (0,0,0)
        self.acceleration = model.drone_stats.drone_acceleration
        self.safety_margin = 5
        self.battery = model.drone_stats.drone_battery_capacity
        self.battery_drain_rate = model.drone_stats.battery_drain_rate
        
        self.strategy = model.strategy
        self.package = None
        
        if cell:
            cell.add_agent(self)
            self.pos = cell.coordinate
        else:
            self.pos = None

        if assigned_packages is None:
            self.assigned_packages = []
        else:
            self.assigned_packages = assigned_packages
            
        self.hub = hub
        self.grid = model.grid

    def step(self):
        if self.cell is None and self.pos is not None:
            self.cell = self.model.grid[self.pos]

        output = self.model.strategy.decide(self)
        action, target = output

        if action == DroneAction.MOVE_TO_CELL:
            if isinstance(target, Cell):
                self.move_towards(target)
        
        elif action == DroneAction.PICKUP_PACKAGE:
            self.pickup(target)
        
        elif action == DroneAction.DROPOFF_PACKAGE:
            self.dropoff()
            
        self.battery -= self.battery_drain_rate

    def get_acceleration(self) -> int:             # later we will add mass to the equation
        return self.acceleration


    def move_to_cell(self, target: Cell):
        if target is None: 
            return
        
        if hex_distance(self.cell, target) > self.speed:
            logging.warning(f"Drone tried to exceed its max speed, max={self.speed}, got={hex_distance(self.cell, target)}")
            return
        
        self.move_to(target)
        self.pos = target.coordinate

    def move_towards(self, target_cell: Cell, end_speed: float = 1):
        """ Move towards the target cell.
        args:
            target_cell: the cell to move towards
            end_speed: % of speed at the end [0 - 1]
        """
        
        cur_speed = hex_vector_len(self.cur_speed_vec)
        breaking_range = sum([speed for speed in range(cur_speed, 0, -self.get_acceleration())])
        if hex_distance(self.cell, target_cell) <= breaking_range + self.safety_margin:
            desired_speed = max(cur_speed - self.get_acceleration() * end_speed, 1)
            print('near', desired_speed)
            print('cur_speed', cur_speed)
            print('get_acceleration', self.get_acceleration())
        else:
            desired_speed = min(cur_speed + self.get_acceleration(), self.speed)
            print('far', desired_speed)
        print('desired_speed', desired_speed)
        print('hex_vector', hex_vector(self.cell, target_cell))
        self.cur_speed_vec = normalize_hex_vector(hex_vector(self.cell, target_cell), desired_speed)
        print('cur_speed_vec', self.cur_speed_vec)
        cur_coords_hex = xy_to_qrs(self.cell.coordinate[0], self.cell.coordinate[1])
        move_coords_hex = tuple(a + b for a, b in zip(cur_coords_hex, self.cur_speed_vec))
        print('move_coords_hex', move_coords_hex)
        move_cell_coords = qer_to_xy(move_coords_hex)
        print('move_cell_coords', move_cell_coords)
        move_cell = self.grid._cells[move_cell_coords]
        self.move_to_cell(move_cell)


    def pickup(self, package: Package):
        if package and package in self.assigned_packages:
            self.assigned_packages.remove(package)
            self.package = package
            package.cell = None
            package.pos = None


    def dropoff(self):
        if self.package:
            package = self.package
            
            package.move_to(self.cell)
            package.pos = self.cell.coordinate
            self.package = None