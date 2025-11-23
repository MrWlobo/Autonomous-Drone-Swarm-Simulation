from __future__ import annotations
from typing import TYPE_CHECKING
from abc import ABC, abstractmethod
from enum import Enum, auto

if TYPE_CHECKING:
    from model.model import DroneModel
    from agents.drone import Drone

class DroneAction(Enum):
    """The set of low-level commands a drone can execute."""
    MOVE_TO_CELL = auto()    # Target: The specific cell to move to
    PICKUP_PACKAGE = auto()  # Target: The package to pick up (in current cell)
    DROPOFF_PACKAGE = auto() # Target: None
    CHARGE = auto()          # Target: None
    WAIT = auto()            # Target: None

class HubAction(Enum):
    """The set of low-level commands a hub can execute."""
    DEPLOY_DRONE = auto()               # Target: None
    COLLECT_DRONE = auto()              # Target: The drone to collect (in current cell)
    CREATE_DELIVERY_REQUEST = auto()    # Target: None
    WAIT = auto()                       # Target: None

class Strategy(ABC):
    def __init__(self, model: DroneModel):
        self.model = model

    @abstractmethod
    def register_drone(self, drone: Drone):
        """Called by a drone at its creation to initialize its state."""
        pass
    
    @abstractmethod
    def decide(self, drone: Drone):
        """Called by a drone every step to get its command."""
        pass

    
    def step(self):
        """
        An optional global step for the strategy,
        run *before* any drones make a decision.
        """
        pass