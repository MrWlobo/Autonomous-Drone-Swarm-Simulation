from mesa.discrete_space import FixedAgent
from agents.package import Package
from agents.drone import Drone
from algorithms.base import HubAction

class Hub(FixedAgent):
    """Station for drones: charges them, sends them on missions, and collects them upon return."""
    def __init__(self, model):
        super().__init__(model.next_id(), model)
        self.package_requests: list[Package] = []
        self.drones: list[Drone] = []

    def step(self):
        action, target = self.strategy.decide(self)

        if action == HubAction.DEPLOY_DRONE:
            pass
        elif action == HubAction.COLLECT_DRONE:
            pass