from mesa.discrete_space import CellAgent

class DropZone(CellAgent):
    """A zone where package should be delivered."""
    def __init__(self, model, cell=None):
        super().__init__( model)
        self.cell = cell