from mesa.agent import Agent
# from model.model import DroneModel
from mesa.discrete_space import CellAgent
from algorithms.base import DroneAction

class Drone(CellAgent):
    def __init__(self, model, cell=None):
        super().__init__(model)
        self.speed = model.drone_stats.drone_speed
        self.battery = model.drone_stats.drone_battery_capacity
        self.battery_drain_rate = model.drone_stats.battery_drain_rate
        
        self.strategy = model.strategy
        self.package = None
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
    def move_to_cell(self, target):
        if target is None:
            return
        if self.cell:
            self.cell.agents.remove(self)
        target.agents.append(self)
        self.cell = target

    def pickup(self, package):
        if package is None:
            return

        self.package = package
        package.cell = None

    def dropoff(self, destination):
        pass

    def charge(self):
        pass
    
    def wait(self):
        pass
