from mesa.agent import Agent
# from model.model import DroneModel
from mesa.discrete_space import CellAgent, Cell
from requests.packages import package

from agents.package import Package
from algorithms.base import DroneAction

class Drone(CellAgent):
    def __init__(self, model, cell=None, assigned_packages: list[Package]=None):
        super().__init__(model)
        self.speed = model.drone_stats.drone_speed
        self.battery = model.drone_stats.drone_battery_capacity
        self.battery_drain_rate = model.drone_stats.battery_drain_rate
        
        self.strategy = model.strategy
        self.package = None
        self.assigned_packages = assigned_packages
        self.cell = cell


    def step(self):
        action, target = self.strategy.decide(self)

        if action == DroneAction.MOVE_TO_CELL:
            target_cell = target
            self.move_to_cell(target_cell)
        
        elif action == DroneAction.PICKUP_PACKAGE:
            self.pickup(target)
        
        elif action == DroneAction.DROPOFF_PACKAGE:
            self.dropoff()
        
        elif action == DroneAction.CHARGE:
            self.charge()
        
        elif action == DroneAction.WAIT:
            self.wait()
            
        self.battery -= self.battery_drain_rate


    # has to validate if the move is legal with the drone's speed etc.
    def move_to_cell(self, target: Cell):
        if target is None:
            return
        self.cell = target

    def pickup(self, package):
        if package is None:
            return

        if self.assigned_packages:
            self.assigned_packages.pop(0)
            self.package = package
            package.cell = None

    def dropoff(self):
        if self.package:
            package = self.package
            package.cell = self.cell
            self.package = None

    def charge(self):
        pass
    
    def wait(self):
        pass
