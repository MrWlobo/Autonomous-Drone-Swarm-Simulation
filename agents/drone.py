from mesa import Agent

class Drone(Agent):
    def __init__(self, model, strategy, battery):
        super().__init__(model.next_id(), model)
        self.strategy = strategy
        self.battery = battery
        self.package = None

    def step(self):
        action, target = self.strategy.decide(self)

        if action == "MOVE":
            self.move_to(target)
        elif action == "PICKUP":
            self.pickup(target)
        elif action == "DROPOFF":
            self.dropoff(target)
        elif action == "CHARGE":
            self.move_to(target)
            self.charge()
        elif action == "WAIT":
            pass

        self.battery -= self.model.drone_stats.battery_drain_rate

    def move_to(self, target_pos):
        pass

    def pickup(self, package):
        pass

    def dropoff(self, destination):
        pass

    def charge(self):
        pass
