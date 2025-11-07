from mesa import Model
from mesa.space import ContinuousSpace
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


class DroneSimulation(Model):
    def __init__(
            self,
            width,
            height,
            num_drones,
            algorithm_name,
            drone_stats
    ):
        super().__init__()
        self.space = ContinuousSpace(width, height, torus=False)
        self.drone_stats = drone_stats
        self.swarm_strategy = get_strategy_instance(algorithm_name, self)

        for i in range(num_drones):
            drone = Drone(
                self,
                strategy=self.swarm_strategy,
                battery=self.drone_stats.drone_battery_capacity
            )

    def step(self):
        pass