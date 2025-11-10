from mesa.discrete_space import FixedAgent

class Obstacle(FixedAgent):
    """An obstacle that drone's cannot fly over."""
    def __init__(self, model):
        super().__init__(model.next_id(), model)