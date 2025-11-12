from mesa.discrete_space import CellAgent, Cell

class DropZone(CellAgent):
    """A zone where package should be delivered."""
    def __init__(self, model, cell: Cell = None):
        super().__init__(model)
        self.cell = cell

    def __eq__(self, other: CellAgent):
        return self.unique_id == other.unique_id
    
    def __hash__(self):
        return hash(self.unique_id)