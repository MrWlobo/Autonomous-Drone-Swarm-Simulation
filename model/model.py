import logging
import math
from pathlib import Path
from mesa import Model
from mesa.discrete_space import HexGrid
from algorithms.helpers import get_algorithm_instance
from mesa.experimental.devs import ABMSimulator
from mesa.space import PropertyLayer

from model.initial_state import InitialStateSetter, RandomInitialStateSetter, get_initial_state_setter_instance
from model.presets.base import Preset
from model.presets.helpers import get_preset_instance

class DroneStats:
    def __init__(
            self,
            drone_speed,
            drone_battery,
            drain_rate
    ):
        self.drone_speed = drone_speed
        self.drone_battery_capacity = drone_battery
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
            drone_speed (int, optional): Drone speed (hex cells per tick). Defaults to 1.
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
            drone_battery=drone_battery,
            drain_rate=drain_rate
        )
        self.num_drones = num_drones
        self.num_packages = num_packages
        self.num_hubs = num_hubs
        self.num_obstacles = num_obstacles
        
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


    def step(self):
        """Execute one simulation step."""
        if hasattr(self.strategy, "step"):
            self.strategy.step()

        self.agents.shuffle_do("step")


    def next_id(self):
        self.unique_id += 1
        return self.unique_id - 1