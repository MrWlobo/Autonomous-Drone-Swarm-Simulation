from agents.drop_zone import DropZone
from agents.package import Package
from algorithms.base import Strategy, DroneAction
from agents.drone import Drone
from agents.hub import Hub


class HubSpawn(Strategy):
    def register_drone(self, drone):
        pass

    def decide(self, agent):
        if isinstance(agent, Drone):
            return self.decide_for_drone(agent)
        elif isinstance(agent, Hub):
            return self.decide_for_hub(agent)
        elif isinstance(agent, Package):
            return None
        elif isinstance(agent, DropZone):
            return None
    
    def decide_for_hub(hub: Hub):
        if len(hub.package_requests) + len(hub.drones) == 0:
            return 


    def grid_init(self, model):
        num_cells = model.num_hubs
        cells = model.random.sample(model.grid.all_cells.cells, k=num_cells)
        used_cells = 0

        Hub.create_agents(
            model=model,
            n=model.num_drones,
            cell=cells[used_cells:used_cells+model.num_hubs],
            )
        used_cells+=model.num_hubs