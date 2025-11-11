from mesa.discrete_space import CellAgent

class Obstacle(CellAgent):
    """An obstacle that drone's cannot fly over."""
    def __init__(self, model):
        super().__init__(model)
        