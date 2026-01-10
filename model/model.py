from __future__ import annotations
from typing import TYPE_CHECKING
import logging
import math
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
from agents.obstacle import Obstacle
from agents.package import Package

if TYPE_CHECKING:
    pass

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
            drain_rate
    ):
        self.drone_speed = drone_speed
        self.drone_acceleration = drone_acceleration
        self.drone_max_ascent_speed = drone_max_ascent_speed,
        self.drone_max_descent_speed = drone_max_descent_speed,
        self.drone_max_altitude = drone_max_altitude
        self.drone_min_altitude = drone_min_altitude
        self.drone_height = drone_height
        self.drone_battery = drone_battery
        self.battery_drain_rate = drain_rate


class DroneModel(Model):
    def __init__(
            self,
            preset_name: str = None,
            width: int = 50,
            height: int = 50,
            num_drones: int = 2,
            num_packages: int = 4,
            num_hubs: int = 5,
            num_obstacles: int = 0,
            initial_state_setter_name: str = "random",
            algorithm_name: str = None,
            drone_speed: int = 1,
            drone_acceleration: int = 1,
            drone_max_ascent_speed: float = 5,
            drone_max_descent_speed: float = 3,
            drone_max_altitude: float = 50,
            drone_min_altitude: float = 20,
            drone_height: float = 0.5,
            drone_battery: int = 1,
            drain_rate: int = 0,
            simulator: ABMSimulator = None,
            background: Path = None,
            show_gridlines: bool = True,
    ):
        """_summary_

        Args:
            preset_name (str, optional): Preset to use. If a preset is selected, all model paramters are overwritten by it.
                                        Used to load predefined city delivery scenarios, see model.presets. Defaults to None.
            width (int, optional): Grid width (number of hex cells). Defaults to 50.
            height (int, optional): Grid height (number of hex cells). Defaults to 50.
            num_drones (int, optional): Number of drones. Defaults to 2.
            num_packages (int, optional): Number of packages. Defaults to 4.
            num_hubs (int, optional): Number of hubs. Defaults to 5.
            num_obstacles (int, optional): Number of obstacles. Defaults to 0.
            initial_state_setter_name (str, optional): Name of an object that defines how the grid and agents should be initialized,
                                                                    see model.initial_state. Defaults to None.
            algorithm_name (str, optional): Name of drone cooperation algorithm to use, check out the algorithms module. Defaults to None.
            drone_speed (int, optional): Drone maximum speed (hex cells per tick). Defaults to 1.
            drone_acceleration (int, optional): Drone acceleration (hex cells per tick). Defaults to 1.
            drone_max_ascent_speed (float, optional): Drone maximum ascent speed
            drone_max_descent_speed (float, optional): Drone maximum descent speed
            drone_max_altitude (float, optional): Drone maximum altitude above ground
            drone_min_altitude (float, optional): Drone minimum altitude above ground
            drone_height (float, optional): Drone height. Defaults to 0.5.
            drone_battery (int, optional): Drone battery capacity and initial value. Defaults to 1.
            drain_rate (int, optional): Drone battery drain rate (units per tick). Defaults to 0.
            simulator (ABMSimulator, optional): Simulato object to use to run the model. Defaults to None.
            background (Path, optional): Background image path. Defaults to None.
            show_gridlines (bool, optional): Whether grid lines should be rendered, useful to improve performance. Defaults to True.
        """
        super().__init__()
        self.width = width
        self.height = height
        
        self.drone_stats: DroneStats = DroneStats(
            drone_speed=drone_speed,
            drone_acceleration=drone_acceleration,
            drone_max_ascent_speed=drone_max_ascent_speed,
            drone_max_descent_speed=drone_max_descent_speed,
            drone_max_altitude=drone_max_altitude,
            drone_min_altitude=drone_min_altitude,
            drone_height=drone_height,
            drone_battery=drone_battery,
            drain_rate=drain_rate
        )
        self.num_drones = num_drones
        self.num_packages = num_packages
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

        # if a preset is provided, all settings (that the preset defines) are over overwritten according to it
        if preset_name is not None:
            preset_obj = get_preset_instance(preset_name)
            
            if isinstance(preset_obj, Preset):
                preset_obj.set_model_params(self)
            elif preset_name != "None":
                logging.warning(f"Preset with name {preset_name} doesn't exist.")
        
        self.grid = HexGrid((self.width, self.height), torus=False, capacity=math.inf, random=self.random)
        
        # height to be interpeted as height above sea level
        self.grid.height_layer = PropertyLayer("height", self.width, self.height, default_value=0, dtype=int)
        
        self.initial_state_setter.set_initial_state(self)
        
        self.datacollector = DataCollector(
            model_reporters={
                "Active Drones": lambda m: len(m.get_drones()),
                "Collisions(%)": lambda m: 0 # TODO
            }
        )


    def get_elevation(self, pos: tuple[int, int]) -> int:
        """Get the elevation at give coords (as returned by cell.coordinate).

        Args:
            pos (tuple[int, int]): Coords of cell (as returned by cell.coordinate) to get the height of.

        Returns:
            int: Elevation at give coordinates.
        """
        return self.grid.height_layer.data[pos]


    def set_elevation(self, pos: tuple[int, int], value: int) -> None:
        """Set the elevation at give coords (as returned by cell.coordinate).

        Args:
            pos (tuple[int, int]): Coords of cell (as returned by cell.coordinate).
            value (int): The desired height.
        """
        self.grid.height_layer.set_cell(pos, value)

    def get_drone_collisions(self, delete_drones=True) -> list[Cell]:
        collision_cells: list[Cell] = []
        delete_drones: set[Drone] = set()
        for drone in self.get_drones():
            if drone.cell is None:
                continue
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
                    if qrs_hex_distance(drone_last_pos, second_drone_last_pos) <= 2:
                        x,y = qrs_to_xy(round_hex_vector(drone_last_pos))
                        print(x,y)
                        cell = self.grid[(x,y)]
                        collision_cells.append(cell)
                        delete_drones.add(drone)
                        delete_drones.add(second_drone)
                        break
                    drone_last_pos = add_hex_vectors(drone_last_pos, drone_speed)
                    second_drone_last_pos = add_hex_vectors(second_drone_last_pos, second_drone_speed)

            if drone.check_for_collision_with_terrain() or drone.check_for_collision_with_obstacle() or drone.check_for_lack_of_energy():
                delete_drones.add(drone)

        if delete_drones:
            for drone in delete_drones:
                drone.destroy()
        return collision_cells

    def create_collisions(self, cells) -> None:   # Create agents to show collisions
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

    def get_obstacles(self) -> AgentSet:
        return self.agents.select(agent_type=Obstacle)

    def step(self):
        """Execute one simulation step."""
        if hasattr(self.strategy, "step"):
            self.strategy.step()

        self.agents.shuffle_do("step")
        collision_cells = self.get_drone_collisions(delete_drones=True)
        self.create_collisions(collision_cells)
        self.datacollector.collect(self)
        

    def next_id(self):
        self.unique_id += 1
        return self.unique_id - 1