from mesa.discrete_space import FixedAgent

class Hub(FixedAgent):
    """A charging station. Drones go here to charge."""
    def __init__(self, model):
        super().__init__(model.next_id(), model)