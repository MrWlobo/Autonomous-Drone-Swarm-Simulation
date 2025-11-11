import math
from mesa import Model
from mesa.discrete_space import OrthogonalMooreGrid
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
            width=10,
            height=10,
            num_drones=None,
            num_packages=None,
            num_hubs=None,
            hub_delivery_request_chance=None,
            algorithm_name=None,
            drone_stats : DroneStats = None,
            simulator: ABMSimulator = None
    ):
        super().__init__()
        self.width = width
        self.height = height
        self.grid = OrthogonalMooreGrid([width, height], torus=False, capacity=math.inf, random=self.random)
        self.drone_stats: DroneStats = drone_stats
        self.num_drones = num_drones
        self.num_packages = num_packages
        self.num_hubs = num_hubs
        self.hub_delivery_request_chance = hub_delivery_request_chance
        self.simulator = simulator
        self.simulator.setup(self)
        # the strategy can access the model's parameters, like drone_stats,
        # from the 'self' argument passed to it
        self.strategy = get_strategy_instance(algorithm_name, self)
        self.unique_id = 1

        self.strategy.grid_init(self)

        


    def step(self):
        """Execute one simulation step."""
        # 1. Allow the strategy to run any global logic
        # (e.g., run a task auction) *before* agents move.
        if hasattr(self.strategy, "step"):
            self.strategy.step()

        # 2. Activate all the agents.
        # Each agent will ask the strategy "What do I do?"
        self.agents.shuffle_do("step")

    def next_id(self):
        self.unique_id += 1
        return self.unique_id-1