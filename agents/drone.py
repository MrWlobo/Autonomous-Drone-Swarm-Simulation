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
            self.model.grid.move_agent(self, target)
        
        elif action == DroneAction.PICKUP_PACKAGE:
            self.pickup(target)
        
        elif action == DroneAction.DROPOFF_PACKAGE:
            self.dropoff()
        
        elif action == DroneAction.CHARGE:
            self.charge()
        
        elif action == DroneAction.WAIT:
            self.wait()
            
        self.battery -= self.model.battery_drain_rate


    # has to validate if the move is legal with the drone's speed etc.
    def move_to(self, target_pos):
        pass

    def pickup(self, package):
        pass

    def dropoff(self, destination):
        pass

    def charge(self):
        pass
    
    def wait(self):
        pass
