from __future__ import annotations
from typing import TYPE_CHECKING
from mesa.discrete_space import CellAgent, Cell
import logging
import random

from mesa.examples.basic.boid_flockers.app import model

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
        self.max_battery = model.drone_stats.drone_battery
        self.battery = model.drone_stats.drone_battery
        
        self.strategy = model.strategy
        self.package = None
        self.cell = cell

        self.in_hub = False
        
        if cell:
            cell.add_agent(self)
            self.altitude = model.get_elevation(self.cell.coordinate) + 50 # Note: altitude reffers to the lowest part of the drone (excluding its package's height)

        if assigned_packages is None:
            self.assigned_packages = []
        else:
            self.assigned_packages = assigned_packages
            # Timestamp packages assigned at initialization
            for pkg in self.assigned_packages:
                if pkg.assigned_time is None:
                    pkg.assigned_time = model.steps
            
        self.hub = hub
        self.grid = model.grid
        self.model: DroneModel = model

        # Constants used to calculate battery drain rate
        self.g = 9.81
        self.air_resistance = 1.225
        self.b = 0.5 * self.air_resistance * self.model.drone_stats.drone_drag_factor * self.model.drone_stats.drone_front_area

        test_distance = self.model.drone_stats.drone_max_travel_distance_empty
        test_mass = self.model.drone_stats.drone_mass
        test_speed = self.model.drone_stats.drone_max_travel_distance_speed
        self.a = ((self.max_battery * test_speed) / test_distance - self.b * test_speed ** 3) / test_mass ** 1.5

    def add_package(self, package: Package) -> None:
        """Assigns a package to the drone and records the assignment time."""
        if package not in self.assigned_packages and not package.assigned:
            self.assigned_packages.append(package)
            package.assigned_time = self.model.steps

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
            self.change_altitude(target)

        elif action == DroneAction.DESCENT:
            self.change_altitude(target)

        elif action == DroneAction.CHARGE:
            if self.in_hub:
                self.charge(target)

        if action not in [DroneAction.REST, DroneAction.CHARGE]:
            pass

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

    def _max_speed_nearby(self, distance):
        if distance <= 5:
            return 1
        else:
            return 1 + (distance) // (16//self.get_acceleration())
        
    def _is_near_hub(self):
        for hub in self.model.get_hubs():
            if hub.cell is None or hub.unique_id == self.unique_id:
                continue
            if hex_distance(self.cell, hub.cell) <= 1:
                return True
        return False

    def _get_repulsive_vector(self, target_Cell: Cell):
        repulsive_vector = (0,0,0)
        drone_altitude_vector = 0
        if self.cell is None:
            return repulsive_vector, drone_altitude_vector
        cur_speed = hex_vector_len(self.cur_speed_vec)
        # breaking_range (sum of arithmetic sequence)
        max_distance_v = self.speed / 2 * math.ceil(self.speed / self.get_acceleration())
        max_distance_h = 15

        breaking_range = (cur_speed + self.get_acceleration()) / 2 * math.ceil(cur_speed / self.get_acceleration())
        breaking_range = round(breaking_range * 1.8) + 1

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
                        drone_altitude_vector = min(altitude_vector, self.max_ascent_speed)
                    else:
                        altitude_vector = weight_h * self.get_acceleration()
                        drone_altitude_vector = - min(altitude_vector, self.max_descent_speed)

        return repulsive_vector, drone_altitude_vector
    
    def _calculate_desired_speed(self, target_cell, end_speed_percentage):
        cur_speed = hex_vector_len(self.cur_speed_vec)
        end_speed = max(round(self.speed * end_speed_percentage), 1)
        
        accel = self.get_acceleration()
        steps_to_slow = math.ceil((cur_speed - end_speed) / accel)
        breaking_range = (cur_speed + end_speed) / 2 * steps_to_slow
        
        near_target = hex_distance(self.cell, target_cell) <= round(breaking_range * 1.8 + cur_speed + 5)

        # dynamic limits of max speed if nearby drones or hubs
        max_speed = self.speed
        for entity in list(self.model.get_drones()) + list(self.model.get_hubs()):
            if entity.cell is None or entity.unique_id == self.unique_id:
                continue
            max_speed = min(max_speed, self._max_speed_nearby(hex_distance(self.cell, entity.cell)))

        if near_target:
            return min(max(cur_speed - accel, end_speed), max_speed)
        else:
            return min(cur_speed + accel, max_speed)
    
    def _get_steering_vector(self, target_cell, desired_speed):
        cur_speed = hex_vector_len(self.cur_speed_vec)
        speed_change = desired_speed - cur_speed
        accel = self.get_acceleration()
        correct_accel = max(accel//2, 1)

        if speed_change > 0:
            target_vec = normalize_hex_vector(hex_vector(self.cell, target_cell), cur_speed)
            correct_vec = sub_hex_vectors(target_vec, self.cur_speed_vec)
            if hex_vector_len(correct_vec) <= correct_accel:
                return normalize_hex_vector(hex_vector(self.cell, target_cell), speed_change)
            return normalize_hex_vector(correct_vec, correct_accel)

        if speed_change < 0:
            return reverse_hex_vector(normalize_hex_vector(self.cur_speed_vec, abs(speed_change)))

        # speed_change == 0 (maintaining speed with course correction)
        target_vec = normalize_hex_vector(hex_vector(self.cell, target_cell), cur_speed)
        correct_vec = sub_hex_vectors(target_vec, self.cur_speed_vec)
        if hex_vector_len(correct_vec) > correct_accel and cur_speed >= self.speed / 2:
            return normalize_hex_vector(correct_vec, correct_accel)
    
        return (0, 0, 0)
    
    def _calculate_altitude_change(self, target_cell, ground_repulsion, initial_repulsion_z):
        z_vector = initial_repulsion_z
        
        if ground_repulsion:
            elevation = self.model.get_elevation(self.cell.coordinate)
            
            # Repulsion from below
            bottom = self.altitude - (self.package.height if self.package else 0)
            min_alt = self.min_altitude + elevation
            if bottom - min_alt < self.altitude_correct_margin:
                weight = 1 - max(bottom - min_alt, 0) / self.altitude_correct_margin
                z_vector += weight * self.max_ascent_speed * 2
            
            # Repulsion from above
            top = self.altitude + self.height
            max_alt = self.max_altitude + elevation
            if max_alt - top < self.altitude_correct_margin:
                weight = 1 - max(max_alt - top, 0) / self.altitude_correct_margin
                z_vector -= weight * self.max_descent_speed

        z_vector = np.clip(z_vector, -self.max_descent_speed, self.max_ascent_speed)
        return z_vector + random.uniform(-0.2, 0.2)

    def _execute_hex_move(self):
        cur_coords_hex = xy_to_qrs(self.cell.coordinate)
        move_coords_hex = add_hex_vectors(cur_coords_hex, self.cur_speed_vec)
        x, y = qrs_to_xy(move_coords_hex)
        move_cell_coords = (np.clip(x, 0, self.grid.width - 1), np.clip(y, 0, self.grid.height - 1))
        move_cell = self.grid._cells[move_cell_coords]
        self.move_to_cell(move_cell)


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

        # 1. calculating speed and direction
        desired_speed = self._calculate_desired_speed(target_cell, end_speed_percentage)
        change_vec = self._get_steering_vector(target_cell, desired_speed)
        is_near_hub = self._is_near_hub()

        # 2. repulsion vectors
        repulsion_vec, repulsion_z = (0,0,0), 0
        if repulsive_vectors and not is_near_hub:
            repulsion_vec, repulsion_z = self._get_repulsive_vector(target_cell)
            change_vec = add_hex_vectors(change_vec, repulsion_vec)
            accel_len = min(hex_vector_len(change_vec), self.get_acceleration())
            change_vec = normalize_hex_vector(change_vec, accel_len)

        # 3. altitude change
        self.altitude += self._calculate_altitude_change(target_cell, ground_repulsion, repulsion_z)

        # 4. move vector change
        new_vec = add_hex_vectors(self.cur_speed_vec, change_vec)
        new_speed = min(hex_vector_len(new_vec), self.speed, desired_speed)
        if hex_vector_len(repulsion_vec) == 0 and desired_speed == 1:
            if not is_near_hub and random.randint(1, 20) == 1:  # chance for random move to avoid being stuck
                neighbors = list(self.cell.get_neighborhood())
                target_cell = random.choice(neighbors)
            self.cur_speed_vec = normalize_hex_vector(hex_vector(self.cell, target_cell), 1)
        else:
            self.cur_speed_vec = normalize_hex_vector(new_vec, min(new_speed+1, self.speed))

        # 5. physical move
        self._execute_hex_move()

        

    def pickup(self, package: Package) -> None:
        if package and package in self.assigned_packages:
            self.package = package
            package.cell = None
            # Fallback: if assigned_time wasn't set earlier, set it now to avoid errors
            if package.assigned_time is None:
                package.assigned_time = self.model.steps
        else:
            print('Cant pick up, package not assigned to drone')

    def dropoff(self) -> None:
        if self.package is None:
            return
        if self.package.drop_zone.cell == self.cell:
            self.assigned_packages.remove(self.package)
            self.package.deliver()
            self.package = None     # Don't delete package, its stored as completed in model

    def get_altitude_above_ground(self):
        return self.altitude - self.model.get_elevation(self.cell.coordinate) - (self.package.height if self.package else 0)

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

    def change_altitude(self, altitude) -> None:
        """
        Function for changing altitude towards wanted one.
        :param altitude: desired altitude (above ground)
        """
        altitude_change = altitude - (self.altitude - self.model.get_elevation(self.cell.coordinate) - (self.package.height if self.package else 0) )
        if altitude_change > 0:
            altitude_change = min(altitude_change, self.max_ascent_speed)
        else:
            altitude_change = max(altitude_change, -self.max_descent_speed)
        self.altitude += altitude_change

    def calculate_energy_drain(self, horizontal_speed, altitude_change, hex_distance: int, hex_size: int =2 ):
        mass = self.model.drone_stats.drone_mass
        if self.package:
            mass += self.package.weight
        distance = hex_distance * hex_size

        if horizontal_speed == 0:
            E_horizontal = 0
        else:
            E_horizontal = ((self.a * mass ** 1.5) / horizontal_speed + self.b * horizontal_speed ** 2) * distance
        E_ascent = (mass * self.g * max(0, altitude_change)) / self.model.drone_stats.drone_climb_efficiency
        E_descent = mass * self.g * max(0, -altitude_change) * self.model.drone_stats.drone_descent_factor

        return E_horizontal + E_ascent + E_descent

    def charge(self, desired_battery_level: int) -> bool:
        """
        Increases the battery level by the drone's charge rate for a single
        charging step and checks whether the desired battery level has been reached.

        The amount added per call is defined by
        `self.model.drone_stats.drone_charge_rate`.

        Args:
            desired_battery_level (int): Target battery level to reach or exceed.

        Returns:
            bool: True if the battery level is greater than or equal to the desired
            battery level after charging; False otherwise.
        """

        self.battery += self.model.drone_stats.drone_charge_rate
        return self.battery >= desired_battery_level