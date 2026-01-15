from __future__ import annotations
from typing import TYPE_CHECKING
import logging
import math
import os
from datetime import datetime
from pathlib import Path
from mesa import Model
from mesa.discrete_space import HexGrid
from algorithms.helpers import get_algorithm_instance
from algorithms.base import DroneAction
from mesa.experimental.devs import ABMSimulator
from mesa.space import PropertyLayer
from mesa.datacollection import DataCollector
from utils.distance import *

from model.initial_state import RandomInitialStateSetter, get_initial_state_setter_instance
from model.presets.base import Preset
from model.presets.helpers import get_preset_instance
from agents.collision import Collision
from mesa.agent import AgentSet
from agents.drone import Drone
from agents.hub import Hub
from agents.drop_zone import DropZone
from agents.package import Package


def compute_avg_delivery_time(model: DroneModel):
    """Calculates the average ticks it took to deliver packages (Assignment -> Delivery)."""
    if not model.completed_deliveries:
        return 0
    
    total_duration = 0
    valid_packages = 0
    for package in model.completed_deliveries:
        if package.delivery_time is not None and package.assigned_time is not None:
            total_duration += (package.delivery_time - package.assigned_time)
            valid_packages += 1
            
    if valid_packages == 0:
        return 0
    return total_duration / valid_packages

def compute_avg_delivery_time_minutes(model: DroneModel):
    return compute_avg_delivery_time(model)/60

def compute_deliveries_per_minute(model: DroneModel):
    return len(model.completed_deliveries)/model.steps*60

def compute_time_per_meter(model: DroneModel):
    if model.meters_traveled == 0:
        return 0
    return model.time_traveled/model.meters_traveled

def compute_collision_percentage(model: DroneModel):
    """Calculates the cumulative percentage of drones that have collided."""
    active = len(model.get_drones())
    total_history = active + model.total_dead_drones
    
    if total_history == 0:
        return 0
    
    return (model.total_collisions / total_history) * 100

def _get_drone_agls(model: DroneModel) -> list[float]:
    """Helper to get list of Above Ground Level (AGL) altitudes for all active drones."""
    drones = [d for d in model.get_drones() if d.cell is not None]
    if not drones:
        return []
    return [d.altitude - model.get_elevation(d.cell.coordinate) for d in drones]

def compute_min_altitude(model: DroneModel):
    agls = _get_drone_agls(model)
    return min(agls) if agls else 0

def compute_mean_altitude(model: DroneModel):
    agls = _get_drone_agls(model)
    return sum(agls) / len(agls) if agls else 0

def compute_max_altitude(model: DroneModel):
    agls = _get_drone_agls(model)
    return max(agls) if agls else 0

def compute_active_drones(model: DroneModel):
    return sum(1 for d in model.get_drones() if d.cell is not None)


class DroneStats:
    def __init__(
            self,
            drone_speed,
            drone_acceleration,
            drone_max_ascent_speed,
            drone_max_descent_speed,
            drone_max_altitude,
            drone_min_altitude,
            drone_height,
            drone_battery,
            drone_safe_battery_level,
            drone_charge_rate,

            drone_mass,
            drone_front_area,
            drone_drag_factor,
            drone_climb_efficiency,
            drone_descent_factor,
            drone_max_travel_distance_empty,
            drone_max_travel_distance_cargo,
            drone_max_travel_distance_speed
    ):
        self.drone_speed = drone_speed
        self.drone_acceleration = drone_acceleration
        self.drone_max_ascent_speed = drone_max_ascent_speed
        self.drone_max_descent_speed = drone_max_descent_speed
        self.drone_max_altitude = drone_max_altitude
        self.drone_min_altitude = drone_min_altitude
        self.drone_height = drone_height
        self.drone_battery = drone_battery
        self.drone_safe_battery_level = drone_safe_battery_level
        self.drone_charge_rate = drone_charge_rate

        self.drone_mass = drone_mass
        self.drone_front_area = drone_front_area
        self.drone_drag_factor = drone_drag_factor
        self.drone_climb_efficiency = drone_climb_efficiency
        self.drone_descent_factor = drone_descent_factor
        self.drone_max_travel_distance_empty = drone_max_travel_distance_empty
        self.drone_max_travel_distance_cargo = drone_max_travel_distance_cargo
        self.drone_max_travel_distance_speed = drone_max_travel_distance_speed


class DroneModel(Model):
    def __init__(
            self,
            preset_name: str = None,
            width: int = 50,
            height: int = 50,
            num_drones: int = 2,
            num_packages: int = 4,
            num_package_clusters: int = 5,
            num_hubs: int = 5,
            num_obstacles: int = 0,
            initial_state_setter_name: str = "random",
            algorithm_name: str = None,
            drone_speed: int = 1,
            drone_acceleration: int = 1,
            drone_max_ascent_speed: float = 5,
            drone_max_descent_speed: float = 3,
            drone_max_altitude: float = 150,
            drone_min_altitude: float = 50,
            drone_height: float = 0.5,
            simulator: ABMSimulator = None,
            background: Path = None,
            show_gridlines: bool = True,
            save_every: int = 10,
            drone_safe_battery_level: int = 10, # Battery percentage
            
            # Plot display flags
            show_active_drones_plot: bool = False,
            show_collisions_plot: bool = False,
            show_completed_deliveries_plot: bool = False,
            show_avg_delivery_time_plot: bool = False,
            show_altitude_plot: bool = False,

            # Stats for battery drain rate calculations
            drone_charge_rate: int = 5700,  # J/s
            drone_battery: int = 14_300_000,  # J
            drone_mass: float = 65, # kg
            drone_front_area: float = 1.06, # m^2
            drone_drag_factor: float = 1.1,
            drone_climb_efficiency: float = 0.8,
            drone_descent_factor: float = 0.3,
            drone_max_travel_distance_empty: int = 28000, # m
            drone_max_travel_distance_cargo: int = 16000, # m
            drone_max_travel_distance_speed: int = 15, # m/s, this value is the one used to determine the 2 variables above, do not confuse with drone_speed
    ):
        super().__init__()
        self.width = width
        self.height = height
        self.save_every = save_every
        
        self.total_collisions = 0 
        self.total_dead_drones = 0
        self.meters_traveled = 0
        self.time_traveled = 0
        
        # Initialize display flags
        self.show_active_drones_plot = show_active_drones_plot
        self.show_collisions_plot = show_collisions_plot
        self.show_completed_deliveries_plot = show_completed_deliveries_plot
        self.show_avg_delivery_time_plot = show_avg_delivery_time_plot
        self.show_altitude_plot = show_altitude_plot

        self.output_dir = None
        self.output_file = None
        
        self.drone_stats: DroneStats = DroneStats(
            drone_speed=drone_speed,
            drone_acceleration=drone_acceleration,
            drone_max_ascent_speed=drone_max_ascent_speed,
            drone_max_descent_speed=drone_max_descent_speed,
            drone_max_altitude=drone_max_altitude,
            drone_min_altitude=drone_min_altitude,
            drone_height=drone_height,
            drone_battery=drone_battery,
            drone_safe_battery_level=drone_safe_battery_level,
            drone_charge_rate=drone_charge_rate,

            drone_mass=drone_mass,
            drone_front_area=drone_front_area,
            drone_drag_factor=drone_drag_factor,
            drone_climb_efficiency=drone_climb_efficiency,
            drone_descent_factor=drone_descent_factor,
            drone_max_travel_distance_empty=drone_max_travel_distance_empty,
            drone_max_travel_distance_cargo=drone_max_travel_distance_cargo,
            drone_max_travel_distance_speed=drone_max_travel_distance_speed

        )
        self.num_drones = num_drones
        self.num_packages = num_packages
        self.num_package_clusters = num_package_clusters
        self.num_hubs = num_hubs
        self.num_obstacles = num_obstacles

        self.completed_deliveries: list[Package] = []
        self.failed_deliveries: list[Package] = []
        
        self.initial_state_setter = get_initial_state_setter_instance(initial_state_setter_name)
        if self.initial_state_setter is None:
            self.initial_state_setter = RandomInitialStateSetter()
        
        self.simulator = simulator
        if self.simulator:
            self.simulator.setup(self)

        self.strategy = get_algorithm_instance(algorithm_name, self)
        self.unique_id = 1
        
        self.background = background
        self.show_gridlines = show_gridlines

        if preset_name is not None:
            preset_obj = get_preset_instance(preset_name)
            
            if isinstance(preset_obj, Preset):
                preset_obj.set_model_params(self)
            elif preset_name != "None":
                logging.warning(f"Preset with name {preset_name} doesn't exist.")

        self.grid = HexGrid((self.width, self.height), torus=False, capacity=math.inf, random=self.random)

        self.grid.height_layer = PropertyLayer("height", self.width, self.height, default_value=0, dtype=int)

        self.initial_state_setter.set_initial_state(self)
        
        self.datacollector = DataCollector(
            model_reporters={
                "Total Drones": lambda m: len(m.get_drones()),
                "Active Drones": compute_active_drones,
                "Collisions": lambda m: m.total_dead_drones,
                "Completed Deliveries": lambda m: len(m.completed_deliveries),
                "Avg Delivery Time [minutes]": compute_avg_delivery_time_minutes,
                "Deliveries Per Minute": compute_deliveries_per_minute,
                "Time Per Meter [seconds]": compute_time_per_meter,
                "Min Altitude": compute_min_altitude,
                "Mean Altitude": compute_mean_altitude,
                "Max Altitude": compute_max_altitude,
                
            }
        )


    def get_elevation(self, pos: tuple[int, int]) -> int:
        return self.grid.height_layer.data[pos]

    def set_elevation(self, pos: tuple[int, int], value: int) -> None:
        self.grid.height_layer.set_cell(pos, value)

    def get_drone_collisions(self, delete_drones=True) -> list[Cell]:
        collision_cells: list[Cell] = []
        
        delete_drones_set: set[Drone] = set()
        collided_drones_set: set[Drone] = set()

        for drone in self.get_drones():
            if drone.cell is None:
                continue
                
            # check for drone-to-drone collisions
            for second_drone in self.get_drones():
                if second_drone.cell is None or drone.unique_id == second_drone.unique_id or drone.last_action != DroneAction.MOVE_TO_CELL:
                    continue
                
                drone_last_pos = sub_hex_vectors(xy_to_qrs(drone.cell.coordinate), drone.cur_speed_vec)
                
                if second_drone.last_action != DroneAction.MOVE_TO_CELL:
                    num_check = hex_vector_len(drone.cur_speed_vec)
                    if num_check == 0:
                        continue
                    second_drone_speed = (0,0,0)
                    second_drone_last_pos = xy_to_qrs(second_drone.cell.coordinate)
                else:
                    num_check = max(hex_vector_len(drone.cur_speed_vec), hex_vector_len(second_drone.cur_speed_vec))
                    if num_check == 0:
                        continue
                    second_drone_speed = divide_hex_vector(second_drone.cur_speed_vec, num_check)
                    second_drone_last_pos = sub_hex_vectors(xy_to_qrs(second_drone.cell.coordinate), second_drone.cur_speed_vec)
                
                drone_speed = divide_hex_vector(drone.cur_speed_vec, num_check)

                for _ in range(num_check + 1):
                    if qrs_hex_distance(drone_last_pos, second_drone_last_pos) <= 2 and drone.check_for_collision_with_drone(second_drone):
                        x,y = qrs_to_xy(round_hex_vector(drone_last_pos))
                        cell = self.grid[(x,y)]
                        collision_cells.append(cell)
                        
                        # mark as collision
                        delete_drones_set.add(drone)
                        delete_drones_set.add(second_drone)
                        collided_drones_set.add(drone)
                        collided_drones_set.add(second_drone)
                        break
                        
                    drone_last_pos = add_hex_vectors(drone_last_pos, drone_speed)
                    second_drone_last_pos = add_hex_vectors(second_drone_last_pos, second_drone_speed)

            # check for terrain collisions
            if drone.check_for_collision_with_terrain():
                delete_drones_set.add(drone)
                collided_drones_set.add(drone)
            
            # check for battery depletion
            elif drone.check_for_lack_of_energy():
                delete_drones_set.add(drone)

        # Update Model Stats
        self.total_collisions += len(collided_drones_set)
        self.total_dead_drones += len(delete_drones_set)

        if delete_drones:
            for drone in delete_drones_set:
                drone.destroy()
        
        return collision_cells

    def create_collisions(self, cells) -> None:
        for cell in cells:
            c = Collision(self, cell=cell)
    
    def get_drop_zones(self) -> AgentSet:
        return self.agents.select(agent_type=DropZone)
    
    def get_packages(self) -> AgentSet:
        return self.agents.select(agent_type=Package)
    
    def get_drones(self) -> AgentSet:
        return self.agents.select(agent_type=Drone)

    def get_hubs(self) -> AgentSet:
        return self.agents.select(agent_type=Hub)

    def step(self):
        """Execute one simulation step."""
        
        if self.save_every > 0 and self.output_dir is None:
            if self.save_every > 0:
                timestamp = datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
                self.output_dir = Path(__file__).parent.parent / "experiments" / f"run_{timestamp}"
                os.makedirs(self.output_dir, exist_ok=True)
                self.output_file = self.output_dir / "model_data.csv"
                logging.info(f"Saving data to {self.output_file} every {self.save_every} steps.")
            else:
                self.output_dir = None
        
        if hasattr(self.strategy, "step"):
            self.strategy.step()

        self.agents.shuffle_do("step")
        collision_cells = self.get_drone_collisions(delete_drones=True)
        self.create_collisions(collision_cells)
        self.datacollector.collect(self)
        
        # Save data to CSV
        if self.output_dir and self.steps > 0 and self.steps % self.save_every == 0:
            df = self.datacollector.get_model_vars_dataframe()
            df.to_csv(self.output_file)

    def next_id(self):
        self.unique_id += 1
        return self.unique_id - 1