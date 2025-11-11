from agents.drop_zone import DropZone
from agents.package import Package
from algorithms.base import Strategy, DroneAction
from agents.drone import Drone


class Dummy(Strategy):
    def register_drone(self, drone):
        pass

    def decide(self, drone):
        if not drone.package and not drone.assigned_packages:
            return DroneAction.WAIT, drone.cell

        elif drone.package and drone.cell == drone.package.drop_zone.cell:
            return DroneAction.DROPOFF_PACKAGE, drone.cell

        elif drone.package is None and drone.assigned_packages:
            package = drone.assigned_packages[0]
            position = package.cell

            new_x, new_y = drone.cell.coordinate
            pack_x, pack_y = position.coordinate

            if new_x != pack_x:
                new_x += int((pack_x - new_x) / abs(pack_x - new_x))
            elif new_y != pack_y:
                new_y += int((pack_y - new_y) / abs(pack_y - new_y))
            else:
                drone.pickup(package)

            target_cell = next(
                (c for c in drone.model.grid.all_cells.cells if c.coordinate == (new_x, new_y)),
                None
            )
            return DroneAction.MOVE_TO_CELL, target_cell

        else:
            drop_zone = drone.package.drop_zone
            position = drop_zone.cell

            new_x, new_y = drone.cell.coordinate
            pack_x, pack_y = position.coordinate

            if new_x != pack_x:
                new_x += int((pack_x - new_x) / abs(pack_x - new_x))
            elif new_y != pack_y:
                new_y += int((pack_y - new_y) / abs(pack_y - new_y))

            target_cell = next(
                (c for c in drone.model.grid.all_cells.cells if c.coordinate == (new_x, new_y)),
                None
            )
            return DroneAction.MOVE_TO_CELL, target_cell
    
    def grid_init(self, model):
        dropzone_cells = model.random.choices(model.grid.all_cells.cells, k=model.num_packages)

        drop_zones = DropZone.create_agents(
            model=model,
            n=model.num_packages,
            cell=dropzone_cells
        )

        packages = Package.create_agents(
            model=model,
            n=model.num_packages,
            cell=model.random.choices(model.grid.all_cells.cells, k=model.num_packages),
            drop_zone=drop_zones
        )

        drones = list(Drone.create_agents(
            model=model,
            n=model.num_drones,
            cell=model.random.choices(model.grid.all_cells.cells, k=model.num_drones),
            assigned_packages=None
            ))

        for i, package in enumerate(packages):
            package.drop_zone = drop_zones[i]

        for i, package in enumerate(packages):
            index = i % len(drones)
            if drones[index].assigned_packages is None:
                drones[index].assigned_packages = [package]
            else:
                drones[index].assigned_packages.append(package)