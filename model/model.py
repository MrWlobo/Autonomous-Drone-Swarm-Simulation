import math
from mesa import Model

from mesa.discrete_space import HexGrid
from algorithms.helpers import get_strategy_instance
from mesa.experimental.devs import ABMSimulator


class DroneStats:
    def __init__(
            self,
            drone_speed,
            drone_battery,
            drain_rate
    ):
        self.drone_speed = drone_speed
        self.drone_battery_capacity = drone_battery
        self.battery_drain_rate = drain_rate


class DroneModel(Model):
    def __init__(
            self,
            width=50,
            height=50,
            num_drones=2,
            num_packages=4,
            num_hubs=5,
            hub_delivery_request_chance=None,
            algorithm_name=None,
            drone_speed: int = 1,
            drone_battery: int = 1,
            drain_rate: int = 0,
            simulator: ABMSimulator = None
    ):
        super().__init__()
        self.width = width
        self.height = height
        
        self.grid = HexGrid((width, height), torus=False, capacity=math.inf, random=self.random)
        
        self.drone_stats: DroneStats = DroneStats(
            drone_speed=drone_speed,
            drone_battery=drone_battery,
            drain_rate=drain_rate
        )
        self.num_drones = num_drones
        self.num_packages = num_packages
        self.num_hubs = num_hubs
        self.hub_delivery_request_chance = hub_delivery_request_chance
        self.simulator = simulator
        
        if self.simulator:
            self.simulator.setup(self)
            
        self.strategy = get_strategy_instance(algorithm_name, self)
        self.unique_id = 1

        self.strategy.grid_init(self)


    def step(self):
        """Execute one simulation step."""
        if hasattr(self.strategy, "step"):
            self.strategy.step()

        self.agents.shuffle_do("step")


    def next_id(self):
        self.unique_id += 1
        return self.unique_id - 1