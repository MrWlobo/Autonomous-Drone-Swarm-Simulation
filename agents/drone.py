from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell
import logging
import random

from agents.package import Package
from algorithms.base import DroneAction
from utils.distance import *
import math
import numpy as np

if TYPE_CHECKING:
    from model.model import DroneModel
    from agents.hub import Hub

class Drone(CellAgent):
    def __init__(self, model: DroneModel, cell: Cell = None, assigned_packages: list[Package] = None, hub: Hub = None):
        super().__init__(model)
        self.speed = model.drone_stats.drone_speed
        self.cur_speed_vec = (0,0,0)
        self.last_action = None
        self.acceleration = model.drone_stats.drone_acceleration
        self.max_ascent_speed = model.drone_stats.drone_max_ascent_speed
        self.max_descent_speed = model.drone_stats.drone_max_descent_speed
        self.max_altitude = model.drone_stats.drone_max_altitude
        self.min_altitude = model.drone_stats.drone_min_altitude
        self.altitude_correct_margin = 5      # start pushing the drone down if it's too close to min/max height
        self.current_ascent_speed = 0
        self.height = model.drone_stats.drone_height
        self.battery = model.drone_stats.drone_battery
        self.battery_drain_rate = model.drone_stats.battery_drain_rate
        
        self.strategy = model.strategy
        self.package = None
        self.cell = cell
        
        if cell:
            cell.add_agent(self)
            self.altitude = model.get_elevation(self.cell.coordinate) + 10 # Note: altitude reffers to the lowest part of the drone (excluding its package's height)

        if assigned_packages is None:
            self.assigned_packages = []
        else:
            self.assigned_packages = assigned_packages
            
        self.hub = hub
        self.grid = model.grid
        self.model: DroneModel = model

    def step(self) -> None:
        output = self.model.strategy.decide(self)
        action, target = output
        self.last_action = action

        if action == DroneAction.MOVE_TO_CELL:
            if isinstance(target, Cell):
                self.move_towards(target)
        
        elif action == DroneAction.PICKUP_PACKAGE:
            self.pickup(target)
        
        elif action == DroneAction.DROPOFF_PACKAGE:
            self.dropoff()

        elif action == DroneAction.DESTROY:
            self.destroy()

        elif action == DroneAction.ASCENT:
            self.ascent()

        elif action == DroneAction.DESCENT:
            self.descent(target)

        if action != DroneAction.REST:
            self.battery -= self.battery_drain_rate

    def __eq__(self, other):
        if other is None:
            return False
        if not isinstance(other, type(self)):
            return False
        return other is not None and self.unique_id == other.unique_id
    
    def __hash__(self):
        return hash(self.unique_id)

    def get_acceleration(self) -> int:             # later we will add mass to the equation
        return self.acceleration

    def move_to_cell(self, target: Cell) -> None:
        if target is None: 
            return
        
        if hex_distance(self.cell, target) > self.speed:
            logging.warning(f"Drone tried to exceed its max speed, max={self.speed}, got={hex_distance(self.cell, target)}")
            return
        
        self.move_to(target)

    def max_speed_nearby(self, distance):
        if distance <= 10:
            return 1
        else:
            return distance // 5


    def get_repulsive_vector(self, target_Cell: Cell):
        repulsive_vector = (0,0,0)
        drone_altitude_vector = 0
        if self.cell is None:
            return repulsive_vector, drone_altitude_vector
        cur_speed = hex_vector_len(self.cur_speed_vec)
        # breaking_range (sum of arithmetic sequence)
        max_distance_v = self.speed / 2 * math.ceil(self.speed / self.get_acceleration())
        max_distance_h = 15

        breaking_range = (cur_speed + self.get_acceleration()) / 2 * math.ceil(cur_speed / self.get_acceleration())
        breaking_range = round(breaking_range * 2.5)

        for other_drone in self.model.get_drones():
            if other_drone.cell is None or other_drone.unique_id == self.unique_id:
                continue
            drone_distance = hex_distance(self.cell, other_drone.cell)
            drone_altitude_difference = self.altitude - other_drone.altitude

            if drone_distance <= breaking_range:
                weight_v = max(1 - drone_distance/max_distance_v, 0)
                repulsive_vector = add_hex_vectors(repulsive_vector, normalize_hex_vector(hex_vector(other_drone.cell, self.cell), weight_v*self.get_acceleration()))
                
            if drone_distance <= breaking_range:
                if abs(drone_altitude_difference) < max_distance_h:
                    weight_h = 1 - abs(drone_altitude_difference)/max_distance_h
                    if drone_altitude_difference >= 0:
                        altitude_vector = weight_h * self.get_acceleration()
                        drone_altitude_vector = min(altitude_vector, self.max_ascent_speed[0])
                    else:
                        altitude_vector = weight_h * self.get_acceleration()
                        drone_altitude_vector = - min(altitude_vector, self.max_descent_speed[0])

        return repulsive_vector, drone_altitude_vector

    def move_towards(self, target_cell: Cell,
                     end_speed_percentage: float = 0,
                     repulsive_vectors: bool = True,
                     ground_repulsion: bool = True
                     ):
        """ Move towards the target cell.
        args:
            target_cell: (Cell) the cell to move towards
            end_speed: (float) % of speed at the end [0 - 1]
            repulsive_vectors: (bool) whether to add repulsive vectors to the movement
                                this includes repulsion vectors from other drones and current cell's terrain
            ground_repulsion: (bool) whether to add repulsion vectors from the ground
        """
        
        cur_speed = hex_vector_len(self.cur_speed_vec)
        end_speed = round(self.speed * end_speed_percentage)
        breaking_range = (cur_speed + end_speed)/2 * math.ceil((cur_speed - end_speed) / self.get_acceleration())
        end_speed = max(end_speed, 1)   # we need to make sure it is at least 1
        near_target = hex_distance(self.cell, target_cell) <= round(breaking_range * 1.8 + cur_speed + 5)

        max_speed = self.speed      # lower max speed if nearby to other drones/hubs
        for other_drone in self.model.get_drones():
            if other_drone.cell is None or other_drone.unique_id == self.unique_id:
                continue
            max_speed = min(max_speed, self.max_speed_nearby(hex_distance(self.cell, other_drone.cell)))
        for hub in self.model.get_hubs():
            if hub.cell is None:
                continue
            max_speed = min(max_speed, self.max_speed_nearby(hex_distance(self.cell, hub.cell)))

        if near_target == False:    # go faster (if possible) if we are far away
            new_speed = min(cur_speed + self.get_acceleration(), self.speed)    
            
        if near_target:             # slow down to end_speed if we are near target cell
            new_speed = max(cur_speed - self.get_acceleration(), end_speed)
        new_speed = min(new_speed, max_speed)
        speed_change =  new_speed - cur_speed
        change_vector = (0,0,0)
        if speed_change > 0:    # speed up towards target
            target_vector = normalize_hex_vector(hex_vector(self.cell, target_cell), cur_speed)
            correct_vector = sub_hex_vectors(target_vector, self.cur_speed_vec)
            if hex_vector_len(correct_vector) <= self.get_acceleration():
                change_vector = normalize_hex_vector(hex_vector(self.cell, target_cell), speed_change)
            else:
                change_vector = normalize_hex_vector(correct_vector, self.get_acceleration())

        elif speed_change < 0:   # slow down, no direction
            change_vector = reverse_hex_vector(normalize_hex_vector(self.cur_speed_vec, abs(speed_change)))

        elif speed_change == 0:
            if cur_speed <= self.get_acceleration():
                change_vector = (0,0,0)
                self.cur_speed_vec = normalize_hex_vector(hex_vector(self.cell, target_cell), cur_speed)
            elif cur_speed >= self.speed / 2:
                target_vector = normalize_hex_vector(hex_vector(self.cell, target_cell), cur_speed)
                correct_vector = sub_hex_vectors(target_vector, self.cur_speed_vec)
                if hex_vector_len(correct_vector) <= self.get_acceleration():
                    change_vector = (0,0,0)
                else:
                    change_vector = normalize_hex_vector(correct_vector, self.get_acceleration())

        drone_altitude_vector = 0

        if repulsive_vectors or ground_repulsion:
            repulsive_vector, drone_altitude_vector = self.get_repulsive_vector(target_cell)

        if repulsive_vectors:   # add repulsive vectors
            change_vector = add_hex_vectors(change_vector, repulsive_vector)
            change_vector_len = min(hex_vector_len(change_vector), self.get_acceleration())
            change_vector = normalize_hex_vector(change_vector, change_vector_len)

        if ground_repulsion:    # push the drone if it's too close to min/max height
            drone_bottom_altitude = self.altitude - (self.package.height if self.package else 0)
            min_altitude = self.min_altitude + self.model.get_elevation(self.cell.coordinate)
            if drone_bottom_altitude - min_altitude < self.altitude_correct_margin:
                weight = 1 - max(drone_bottom_altitude - min_altitude, 0) / self.altitude_correct_margin
                drone_altitude_vector += weight * self.max_ascent_speed[0] * 2        # add more weight to the ascent
            
            drone_top_altitude = self.altitude + self.height
            max_altitude = self.max_altitude + self.model.get_elevation(self.cell.coordinate)
            if max_altitude - drone_top_altitude < self.altitude_correct_margin:
                weight = 1 - max(max_altitude - drone_top_altitude, 0) / self.altitude_correct_margin
                drone_altitude_vector -= weight * self.max_descent_speed[0]

        drone_altitude_vector = np.clip(drone_altitude_vector, -self.max_descent_speed[0], self.max_ascent_speed[0])
        drone_altitude_vector += random.uniform(-0.2, 0.2)    # add some randomness to the height vector

        self.altitude += drone_altitude_vector
        new_speed = min(hex_vector_len(add_hex_vectors(self.cur_speed_vec, change_vector)), self.speed)
        self.cur_speed_vec = normalize_hex_vector(add_hex_vectors(self.cur_speed_vec, change_vector), new_speed)

        cur_coords_hex = xy_to_qrs(self.cell.coordinate)
        move_coords_hex = add_hex_vectors(cur_coords_hex, self.cur_speed_vec)
        x, y = qrs_to_xy(move_coords_hex)
        move_cell_coords = (np.clip(x, 0, self.grid.width - 1), np.clip(y, 0, self.grid.height - 1))
        move_cell = self.grid._cells[move_cell_coords]
        self.move_to_cell(move_cell)

    def pickup(self, package: Package) -> None:
        if package and package in self.assigned_packages:
            self.package = package
            package.cell = None

    def dropoff(self) -> None:
        if self.package is None:
            return
        if self.package.drop_zone.cell == self.cell:
            self.assigned_packages.remove(self.package)
            self.package.deliver()
            self.package = None     # Don't delete package, its stored as completed in model

    def check_for_collision_with_drone(self, other: Drone) -> bool:
        if self.altitude > other.altitude:
            higher_drone_bottom = self.altitude - (self.package.height if self.package else 0)
            lower_drone_top = other.altitude + other.height
        else:
            higher_drone_bottom = other.altitude - (other.package.height if other.package else 0)
            lower_drone_top = self.altitude + self.height

        return higher_drone_bottom <= lower_drone_top

    def check_for_collision_with_terrain(self) -> bool:
        return self.altitude < self.model.get_elevation(self.cell.coordinate)

    def check_for_lack_of_energy(self) -> bool:
        return self.battery <= 0

    def destroy(self) -> None:
        if self.hub is not None:
            self.hub.incomming_drones.remove(self)
        if self.package is not None:
            self.model.failed_deliveries.append(self.package)   # Don't delete package, its stored as completed in model
        self.model.agents.remove(self)  
        logging.warning(f"Drone destroyed at {self.cell.coordinate}, id: {self.unique_id}, altitide: {self.altitude}")

    def ascent(self) -> None:
        self.altitude += self.max_ascent_speed

    def descent(self, elevation) -> None:
        new_altitude = self.altitude - self.max_descent_speed
        if new_altitude < elevation:
            new_altitude = elevation
        self.altitude = new_altitude

    def change_altitude(self, altitude) -> None:
        """
        Function for changing altitude towards wanted one.
        :param altitude: desired altitude (above ground)
        """
        altitude_change = altitude + self.model.get_elevation(self.cell.coordinate) - self.altitude 
        if altitude_change > 0:
            altitude_change = min(altitude_change, self.max_ascent_speed)
        else:
            altitude_change = max(altitude_change, -self.max_descent_speed)
        self.altitude = altitude + altitude_change