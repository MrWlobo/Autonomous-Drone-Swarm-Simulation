from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell
import logging

from agents.package import Package
from algorithms.base import DroneAction
from utils.distance import hex_distance

if TYPE_CHECKING:
    from model.model import DroneModel
    from agents.hub import Hub

class Drone(CellAgent):
    def __init__(self, model: DroneModel, cell: Cell = None, assigned_packages: list[Package] = None, hub: Hub = None):
        super().__init__(model)
        self.speed = model.drone_stats.drone_speed
        self.max_ascent_speed = model.drone_stats.drone_max_ascent_speed
        self.max_descent_speed = model.drone_stats.drone_max_descent_speed
        self.current_ascent_speed = 0
        self.current_descent_speed = 0
        self.height = model.drone_stats.drone_height
        self.battery = model.drone_stats.drone_battery
        self.battery_drain_rate = model.drone_stats.battery_drain_rate
        
        self.strategy = model.strategy
        self.package = None
        
        if cell:
            cell.add_agent(self)
            self.pos = cell.coordinate
            self.altitude = model.get_elevation(self.pos) + 10 # Note: altitude reffers to the lowest part of the drone (excluding its package's height)
        else:
            self.pos = None

        if assigned_packages is None:
            self.assigned_packages = []
        else:
            self.assigned_packages = assigned_packages
            
        self.hub = hub

    def step(self) -> None:
        if self.cell is None and self.pos is not None:
            self.cell = self.model.grid[self.pos]

        output = self.model.strategy.decide(self)
        action, target = output

        if action == DroneAction.MOVE_TO_CELL:
            if isinstance(target, Cell):
                self.move_to_cell(target)
        
        elif action == DroneAction.PICKUP_PACKAGE:
            self.pickup(target)
        
        elif action == DroneAction.DROPOFF_PACKAGE:
            self.dropoff()

        elif action == DroneAction.DESTROY:
            self.destroy()

        elif action == DroneAction.ASCENT:
            self.ascent()

        elif action == DroneAction.DESCENT:
            self.descent(target)

        if action != DroneAction.REST:
            self.battery -= self.battery_drain_rate


    def move_to_cell(self, target: Cell) -> None:
        if target is None: 
            return
        
        if hex_distance(self.cell, target) > self.speed:
            logging.warning(f"Drone tried to exceed its max speed, max={self.speed}, got={hex_distance(self.cell, target)}")
            return
        
        self.move_to(target)
        self.pos = target.coordinate


    def pickup(self, package: Package) -> None:
        if package and package in self.assigned_packages:
            self.assigned_packages.remove(package)
            self.package = package
            package.cell = None
            package.pos = None


    def dropoff(self) -> None:
        if self.package:
            package = self.package
            
            package.move_to(self.cell)
            package.pos = self.cell.coordinate
            self.package = None

    def check_for_collision_with_drone(self, other: Drone) -> bool:
        if self.altitude > other.altitude:
            higher_drone_bottom = self.altitude - (self.package.height if self.package else 0)
            lower_drone_top = other.altitude + other.height
        else:
            higher_drone_bottom = other.altitude - (other.package.height if other.package else 0)
            lower_drone_top = self.altitude + self.height

        return higher_drone_bottom <= lower_drone_top

    def check_for_collision_with_terrain(self) -> bool:
        return self.altitude < self.model.get_elevation(self.pos)

    def check_for_lack_of_energy(self) -> bool:
        return self.battery <= 0

    def destroy(self) -> None:
        self.model.agents.remove(self)
        logging.warning(f"Drone destroyed at {self.pos}")

    def ascent(self) -> None:
        self.altitude += self.max_ascent_speed

    def descent(self, elevation) -> None:
        new_altitude = self.altitude - self.max_descent_speed
        if new_altitude < elevation:
            new_altitude = elevation
        self.altitude = new_altitude