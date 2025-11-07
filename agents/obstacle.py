from mesa import Agent

class Obstacle(Agent):
    """An obstacle that drone's cannot fly over."""
    def __init__(self, model):
        super().__init__(model.next_id(), model)