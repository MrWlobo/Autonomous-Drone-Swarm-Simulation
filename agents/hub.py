from mesa import Agent

class Hub(Agent):
    """A charging station. Drones go here to charge."""
    def __init__(self, model):
        super().__init__(model.next_id(), model)