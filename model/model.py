from mesa import Model
from mesa.space import HexMultiGrid
from agents.drone import Drone
from algorithms.helpers import get_strategy_instance


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
            width,
            height,
            num_drones,
            algorithm_name,
            drone_stats : DroneStats
    ):
        super().__init__()
        self.grid = HexMultiGrid(width, height, torus=False)
        self.drone_stats: DroneStats = drone_stats
        # the strategy can access the model's parameters, like drone_stats,
        # from the 'self' argument passed to it
        self.strategy = get_strategy_instance(algorithm_name, self)

        Drone.create_agents(model=self, n=num_drones)


    def step(self):
        """Execute one simulation step."""
        # 1. Allow the strategy to run any global logic
        # (e.g., run a task auction) *before* agents move.
        if hasattr(self.strategy, "step"):
            self.strategy.step()

        # 2. Activate all the agents.
        # Each agent will ask the strategy "What do I do?"
        self.agents.shuffle_do("step")