from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell
import logging

from agents.package import Package
from algorithms.base import DroneAction
from utils.distance import *
from math import ceil

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
        self.cell = cell
        
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
        self.model = model

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

    def get_repulsive_vector(self, drone_cell: Cell):
        repulsive_vector = (0,0,0)
        drone_height_difference = 0
        cur_speed = hex_vector_len(self.cur_speed_vec)
        # breaking_range (sum of arithmetic sequence)
        distance = self.speed / 2 * ceil(self.speed / self.get_acceleration())

        breaking_range = cur_speed / 2 * ceil(cur_speed / self.get_acceleration())
        breaking_range = round(breaking_range * 2 + 5)
        for other_drone in self.model.drones:
            if other_drone.unique_id == self.unique_id:
                continue
            drone_distance = hex_distance(drone_cell, other_drone.cell)
            drone_height_difference = 0 # TODO implement this
            if drone_distance <= breaking_range:
                weight = 1 - drone_distance/distance
                repulsive_vector = add_hex_vectors(repulsive_vector, normalize_hex_vector(hex_vector(other_drone.cell, drone_cell), weight*self.get_acceleration()))

        return repulsive_vector, drone_height_difference

    def move_towards(self, target_cell: Cell,
                     end_speed_percentage: float = 0,
                     repulsive_vectors: bool = True):
        """ Move towards the target cell.
        args:
            target_cell: the cell to move towards
            end_speed: % of speed at the end [0 - 1]
            repulsive_vectors: whether to add repulsive vectors to the movement
        """
        
        cur_speed = hex_vector_len(self.cur_speed_vec)
        end_speed = round(self.speed * end_speed_percentage)
        breaking_range = (cur_speed + end_speed)/2 * ceil((cur_speed - end_speed) / self.get_acceleration())
        end_speed = max(end_speed, 1)   # we need to make sure it is at least 1
        near_target = hex_distance(self.cell, target_cell) <= round(breaking_range * 1.5 + cur_speed + 5)

        if near_target == False:    # go faster (if possible) if we are far away
            new_speed = min(cur_speed + self.get_acceleration(), self.speed)    
            
        if near_target:             # slow down to end_speed if we are near target cell
            new_speed = max(cur_speed - self.get_acceleration(), end_speed)

        speed_change =  new_speed - cur_speed
        # print(f"id: {self.unique_id}, cur_speed: {cur_speed}, new_speed: {new_speed}, speed_change: {speed_change}, end_speed: {end_speed}, breaking_range: {breaking_range}, near_target: {near_target}")
        if speed_change > 0:    # speed up towards target
            target_vector = normalize_hex_vector(hex_vector(self.cell, target_cell), cur_speed)
            correct_vector = sub_hex_vectors(target_vector, self.cur_speed_vec)
            if hex_vector_len(correct_vector) <= self.get_acceleration():
                change_vector = normalize_hex_vector(hex_vector(self.cell, target_cell), speed_change)
            else:
                change_vector = normalize_hex_vector(correct_vector, self.get_acceleration())

        elif speed_change < 0:   # slow down, no direction
            change_vector = reverse_hex_vector(normalize_hex_vector(self.cur_speed_vec, abs(speed_change)))

        elif speed_change == 0:
            if cur_speed <= self.get_acceleration():
                change_vector = (0,0,0)
                self.cur_speed_vec = normalize_hex_vector(hex_vector(self.cell, target_cell), cur_speed)
            else:
                change_vector = (0,0,0)
                pass # TODO correct trajectory towards target (avoid orbiting around target)

        if repulsive_vectors:
            repulsive_vector, height_difference = self.get_repulsive_vector(target_cell)
            # print(f"id: {self.unique_id}, repulsive_vector: {repulsive_vector}")
            change_vector = add_hex_vectors(change_vector, repulsive_vector)
            change_vector_len = min(hex_vector_len(change_vector), self.get_acceleration())
            change_vector = normalize_hex_vector(change_vector, change_vector_len)
        
        new_speed = min(hex_vector_len(add_hex_vectors(self.cur_speed_vec, change_vector)), self.speed)
        self.cur_speed_vec = normalize_hex_vector(add_hex_vectors(self.cur_speed_vec, change_vector), new_speed)

        cur_coords_hex = xy_to_qrs(self.cell.coordinate)
        move_coords_hex = add_hex_vectors(cur_coords_hex, self.cur_speed_vec)
        move_cell_coords = qer_to_xy(move_coords_hex)
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