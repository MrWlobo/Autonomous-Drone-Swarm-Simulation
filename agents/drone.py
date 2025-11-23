from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell

from agents.package import Package
from algorithms.base import DroneAction

if TYPE_CHECKING:
    from model.model import DroneModel
    from agents.hub import Hub

class Drone(CellAgent):
    def __init__(self, model: DroneModel, cell: Cell = None, assigned_packages: list[Package] = None, hub: Hub = None):
        super().__init__(model)
        self.speed = model.drone_stats.drone_speed
        self.battery = model.drone_stats.drone_battery_capacity
        self.battery_drain_rate = model.drone_stats.battery_drain_rate
        
        self.strategy = model.strategy
        self.package = None
        
        if cell:
            cell.add_agent(self)
            self.pos = cell.coordinate
        else:
            self.pos = None

        self.assigned_packages = assigned_packages
        self.hub = hub

    def step(self):
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
            
        self.battery -= self.battery_drain_rate

    def move_to_cell(self, target: Cell):
        print(target)
        if target is None: return
        
        self.move_to(target)
        self.pos = target.coordinate

    def pickup(self, package: Package):
        if package and package in self.assigned_packages:
            self.assigned_packages.remove(package)
            self.package = package
            package.remove()
            package.pos = None

    def dropoff(self):
        if self.package:
            package = self.package
            self.cell.add_agent(package)
            package.pos = self.cell.coordinate
            self.package = None