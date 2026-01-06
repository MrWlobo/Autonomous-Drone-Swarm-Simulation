from mesa.discrete_space import CellAgent, Cell

class DropZone(CellAgent):
    """A zone where package should be delivered."""
    def __init__(self, model, cell: Cell = None):
        super().__init__(model)
        self.cell = cell
        
        if cell:
            cell.add_agent(self)

    def __eq__(self, other):
        if other is None:
            return False
        if not isinstance(other, type(self)):
            return False
        return other is not None and self.unique_id == other.unique_id
    
    def __hash__(self):
        return hash(self.unique_id)