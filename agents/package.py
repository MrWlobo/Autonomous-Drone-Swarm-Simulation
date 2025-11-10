from mesa.discrete_space import CellAgent

class Package(CellAgent):
    """A package to be delivered to a destination."""
    def __init__(self, model):
        super().__init__(model.next_id(), model)