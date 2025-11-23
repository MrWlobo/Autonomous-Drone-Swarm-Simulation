import math
from mesa import Model
from mesa.discrete_space import HexGrid
from algorithms.helpers import get_algorithm_instance
from mesa.experimental.devs import ABMSimulator

from model.initial_state import InitialStateSetter, RandomInitialStateSetter

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
            width: int = 50,
            height: int = 50,
            num_drones: int = 2,
            num_packages: int = 4,
            num_hubs: int = 5,
            num_obstacles: int = 0,
            initial_state_setter: InitialStateSetter = None,
            algorithm_name: str = None,
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
        self.num_obstacles = num_obstacles
        
        if initial_state_setter is None:
            self.initial_state_setter = RandomInitialStateSetter()
        else:
            self.initial_state_setter = initial_state_setter
        
        self.simulator = simulator
        if self.simulator:
            self.simulator.setup(self)
            
        self.strategy = get_algorithm_instance(algorithm_name, self)
        self.unique_id = 1

        self.initial_state_setter.set_initial_state(self)


    def step(self):
        """Execute one simulation step."""
        if hasattr(self.strategy, "step"):
            self.strategy.step()

        self.agents.shuffle_do("step")


    def next_id(self):
        self.unique_id += 1
        return self.unique_id - 1