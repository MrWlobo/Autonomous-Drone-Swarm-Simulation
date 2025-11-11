from mesa.discrete_space import CellAgent

class Package(CellAgent):
    """A package to be delivered to a destination."""
    def __init__(self, model, cell=None, drop_zone=None):
        super().__init__(model)
        self.cell = cell
        self.drop_zone = drop_zone

    def __eq__(self, other):

        return other != None and self.unique_id == other.unique_id
    
    def __hash__(self):
        return hash(self.unique_id)
    