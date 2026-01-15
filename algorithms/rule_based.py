from __future__ import annotations
from typing import TYPE_CHECKING, Optional
import random

from algorithms.base import Strategy, DroneAction, HubAction
from agents.drone import Drone
from agents.hub import Hub
from agents.package import Package
from utils.distance import hex_distance, qrs_to_xy, xy_to_qrs, round_hex_vector, hex_vector, add_hex_vectors, normalize_hex_vector

if TYPE_CHECKING:
    from model.model import DroneModel
    from mesa.discrete_space import Cell


class RuleBased(Strategy):
    """Implements Task-driven Rule-Based (Task-RB) Strategy."""
    
    SENSOR_RANGE = 20         # Range to perceive packages
    PICKUP_LOCK_RANGE = 2     # Distance to lock onto a specific target
    CROWDING_THRESHOLD = 2    # Ignore packages if they are already crowded
    
    SAFE_LAUNCH_RADIUS = 3
    CENTROID_CONVERGENCE_RADIUS = 5 

    def __init__(self, model: DroneModel) -> None:
        super().__init__(model)
        self.claimed_package_ids: set[int] = set()

    def step(self):
        completed_ids = {p.unique_id for p in self.model.completed_deliveries}
        self.claimed_package_ids = self.claimed_package_ids - completed_ids

    def register_drone(self, drone: Drone):
        pass

    def decide(self, agent: Drone | Hub) -> tuple[DroneAction | HubAction, object]:
        if isinstance(agent, Drone):
            return self._decide_drone(agent)
        elif isinstance(agent, Hub):
            return self._decide_hub(agent)
        return None, None

    def _decide_hub(self, hub: Hub) -> tuple[HubAction, object]:
        for agent in hub.cell.agents:
            if isinstance(agent, Drone):
                if not agent.package and not agent.assigned_packages:
                    return HubAction.COLLECT_DRONE, agent

        if hub.stored_drones and Hub.package_requests:
            airspace_clear = True
            for drone in self.model.get_drones():
                if drone.cell is not None:
                    if hex_distance(hub.cell, drone.cell) <= self.SAFE_LAUNCH_RADIUS:
                        airspace_clear = False
                        break
            
            if airspace_clear:
                return HubAction.DEPLOY_DRONE, None

        return HubAction.WAIT, None

    def _decide_drone(self, drone: Drone) -> tuple[DroneAction, object]:
        if drone.cell is None:
            return DroneAction.WAIT, None

        if not drone.package and drone.hub and drone.hub.cell:
            dist_to_hub = hex_distance(drone.cell, drone.hub.cell)
            if dist_to_hub <= self.SAFE_LAUNCH_RADIUS:
                escape_vec = hex_vector(drone.hub.cell, drone.cell)
                
                if escape_vec == (0,0,0):
                    neighbors = list(drone.cell.get_neighborhood())
                    return DroneAction.MOVE_TO_CELL, random.choice(neighbors)
                
                target_hex = add_hex_vectors(xy_to_qrs(drone.cell.coordinate), escape_vec)
                target_cell = self._hex_to_cell(target_hex)
                return DroneAction.MOVE_TO_CELL, target_cell

        if drone.package:
            target = drone.package.drop_zone.cell
            if drone.cell == target:
                return DroneAction.DROPOFF_PACKAGE, None
            return DroneAction.MOVE_TO_CELL, target

        if drone.assigned_packages:
            target_pkg = drone.assigned_packages[0]
            self.claimed_package_ids.add(target_pkg.unique_id)
            if drone.cell == target_pkg.cell:
                return DroneAction.PICKUP_PACKAGE, target_pkg
            return DroneAction.MOVE_TO_CELL, target_pkg.cell

        visible_packages = self._get_visible_packages(drone)
        
        if not visible_packages:
            return self._return_to_hub_or_roam(drone)

        closest_pkg, closest_dist = self._get_closest_package(drone, visible_packages)
        
        should_dive = closest_dist <= self.CENTROID_CONVERGENCE_RADIUS

        if closest_dist <= self.PICKUP_LOCK_RANGE or should_dive:
            self.claimed_package_ids.add(closest_pkg.unique_id)
            if closest_pkg in Hub.package_requests:
                Hub.package_requests.remove(closest_pkg)
            
            if closest_dist == 0:
                return DroneAction.PICKUP_PACKAGE, closest_pkg
            else:
                return DroneAction.MOVE_TO_CELL, closest_pkg.cell

        centroid_cell = self._calculate_centroid_attraction(drone, visible_packages)
        if centroid_cell:
            return DroneAction.MOVE_TO_CELL, centroid_cell
            
        return DroneAction.WAIT, None

    def _hex_to_cell(self, qrs):
        """Helper to safely convert hex coords back to a grid cell object."""
        xy = qrs_to_xy(qrs)
        w, h = self.model.grid.width, self.model.grid.height
        clamped_x = int(max(0, min(xy[0], w - 1)))
        clamped_y = int(max(0, min(xy[1], h - 1)))
        return self.model.grid[(clamped_x, clamped_y)]

    def _get_visible_packages(self, drone: Drone) -> list[Package]:
        visible = []
        for pkg in self.model.get_packages():
            if pkg.unique_id in self.claimed_package_ids or pkg.cell is None:
                continue
            
            dist = hex_distance(drone.cell, pkg.cell)
            if dist <= self.SENSOR_RANGE:
                drones_nearby = 0
                for other_drone in self.model.get_drones():
                    if other_drone.cell and hex_distance(other_drone.cell, pkg.cell) < 3:
                        drones_nearby += 1
                
                if drones_nearby < self.CROWDING_THRESHOLD:
                    visible.append(pkg)
        return visible

    def _calculate_centroid_attraction(self, drone: Drone, packages: list[Package]) -> Cell | None:
        if not packages:
            return None
            
        sum_q, sum_r, sum_s = 0, 0, 0
        for pkg in packages:
            q, r, s = xy_to_qrs(pkg.cell.coordinate)
            sum_q += q
            sum_r += r
            sum_s += s
            
        count = len(packages)
        avg_q = sum_q / count
        avg_r = sum_r / count
        avg_s = sum_s / count
        
        jitter = 0.5
        avg_q += random.uniform(-jitter, jitter)
        avg_r += random.uniform(-jitter, jitter)
        avg_s += random.uniform(-jitter, jitter)

        center_hex = round_hex_vector((avg_q, avg_r, avg_s))
        return self._hex_to_cell(center_hex)

    def _get_closest_package(self, drone: Drone, packages: list[Package]) -> tuple[Optional[Package], float]:
        best = None
        min_dist = float('inf')
        for pkg in packages:
            d = hex_distance(drone.cell, pkg.cell)
            if d < min_dist:
                min_dist = d
                best = pkg
        return best, min_dist

    def _return_to_hub_or_roam(self, drone: Drone):
        if random.random() < 0.2:
            neighbors = list(drone.cell.get_neighborhood())
            if neighbors:
                random_cell = random.choice(neighbors)
                return DroneAction.MOVE_TO_CELL, random_cell

        target_hub = drone.hub
        if not target_hub:
            min_dist = float('inf')
            for hub in self.model.get_hubs():
                if hub.cell:
                    dist = hex_distance(drone.cell, hub.cell)
                    if dist < min_dist:
                        min_dist = dist
                        target_hub = hub
        
        if target_hub and target_hub.cell:
            if drone.cell != target_hub.cell:
                return DroneAction.MOVE_TO_CELL, target_hub.cell
        return DroneAction.WAIT, None