import math
from mesa import Model
from mesa.discrete_space import OrthogonalMooreGrid
from agents.drone import Drone
from agents.package import Package
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
            num_drones=2,
            num_packages=2,
            algorithm_name='test',
            drone_stats : DroneStats = None,
            simulator: ABMSimulator = None
    ):
        super().__init__()
        self.width = width
        self.height = height
        self.grid = OrthogonalMooreGrid([width, height], torus=False, capacity=math.inf, random=self.random)
        self.drone_stats: DroneStats = drone_stats
        self.simulator = simulator
        self.simulator.setup(self)
        # the strategy can access the model's parameters, like drone_stats,
        # from the 'self' argument passed to it
        self.strategy = get_strategy_instance(algorithm_name, self)
        self.unique_id = 1

        Drone.create_agents(
            model=self,
            n=num_drones,
            cell=self.random.choices(self.grid.all_cells.cells, k=num_drones),
            )

        Package.create_agents(
            model=self,
            n=num_packages,
            cell=self.random.choices(self.grid.all_cells.cells, k=num_packages),
            )


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