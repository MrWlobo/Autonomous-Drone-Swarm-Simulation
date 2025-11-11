from mesa.discrete_space import CellAgent
from agents.package import Package
from agents.drone import Drone
from algorithms.base import HubAction
from agents.drop_zone import DropZone

class Hub(CellAgent):
    """Station for drones: charges them, sends them on missions, and collects them upon return."""

    def __init__(self, model, cell=None):
        super().__init__(model)
        self.package_requests: list[Package] = []
        self.active_drones: list[Drone] = []
        self.model = model
        self.cell = cell
    
    def __eq__(self, other):
        return other != None and self.unique_id == other.unique_id
    
    def __hash__(self):
        return hash(self.unique_id)

    def step(self):
        action, target = self.model.strategy.decide(self)
        if action == HubAction.DEPLOY_DRONE:
            drones = Drone.create_agents(
                model=self.model,
                n=1,
                cell=self.cell,
                assigned_packages=[list(self.package_requests)],
                hub=self
            )
            self.package_requests.clear()
            self.active_drones += list(drones)
        elif action == HubAction.COLLECT_DRONE:
            self.active_drones.remove(target)
            target.remove()
        elif action == HubAction.CREATE_DELIVERY_REQUEST:
            empty_cells = [c for c in self.model.grid.all_cells.cells if len(c.agents) == 0]
            if len(empty_cells) < 2:
                return
            self.model.random.shuffle(empty_cells)
            packages = Package.create_agents(
                model=self.model,
                n=1,
                cell=empty_cells[0],
                drop_zone=DropZone(self.model, empty_cells[1])
            )
            self.package_requests+=list(packages)
        elif action == HubAction.WAIT:
            pass