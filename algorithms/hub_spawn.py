from agents.drop_zone import DropZone
from agents.package import Package
from algorithms.base import Strategy, HubAction, DroneAction
from mesa.discrete_space import Cell
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
            return (None, None)
        elif isinstance(agent, DropZone):
            return (None, None)
        return (None, None)
        
    def decide_for_drone(self, drone: Drone):
        if not drone.package and not drone.assigned_packages:
            return self.move_towards(drone, drone.hub.cell)

        elif drone.package and drone.cell == drone.package.drop_zone.cell:
            return DroneAction.DROPOFF_PACKAGE, drone.cell
        
        elif not drone.package and drone.cell == drone.assigned_packages[0].cell:
            package = drone.assigned_packages[0]
            return DroneAction.PICKUP_PACKAGE, package
        
        elif drone.package:
            return self.move_towards(drone, drone.package.drop_zone.cell)
        
        elif not drone.package:
            return self.move_towards(drone, drone.assigned_packages[0].cell)
    
    def decide_for_hub(self, hub: Hub):
        if len(hub.package_requests) + len(hub.active_drones) == 0:
            if hub.model.random.randint(1, 100) <=5:
                return HubAction.CREATE_DELIVERY_REQUEST, None
            else:
                return HubAction.WAIT, None
        elif len(hub.package_requests) != 0:
            return HubAction.DEPLOY_DRONE, None
        elif len(hub.active_drones) != 0:
            for drone in hub.active_drones:
                if drone.cell.coordinate == hub.cell.coordinate and not drone.assigned_packages and drone.package==None:
                    return HubAction.COLLECT_DRONE, drone
        return HubAction.WAIT, None


    def move_towards(self, drone: Drone, cell: Cell):
        drone_x, drone_y = drone.cell.coordinate
        dest_x, dest_y = cell.coordinate
        d_x = dest_x-drone_x
        d_y = dest_y-drone_y

        if abs(d_x) >= abs(d_y):
            move_x=0
            if d_x > 0:
                move_x=1
            elif d_x < 0:
                move_x=-1
            x,y = drone.cell.coordinate
            cell = drone.model.grid._cells.get((x+move_x, y))
            return DroneAction.MOVE_TO_CELL, cell  
        else:
            move_y=0
            if d_y > 0:
                move_y=1
            elif d_y < 0:
                move_y=-1
            x,y = drone.cell.coordinate
            cell = drone.model.grid._cells.get((x, y+move_y))
            return DroneAction.MOVE_TO_CELL, cell


    def grid_init(self, model):
        num_cells = model.num_hubs
        cells = model.random.sample(model.grid.all_cells.cells, k=num_cells)
        used_cells = 0

        Hub.create_agents(
            model=model,
            n=model.num_hubs,
            cell=cells[used_cells:used_cells+model.num_hubs],
            )
        used_cells+=model.num_hubs