from __future__ import annotations
from typing import TYPE_CHECKING
from algorithms.dummy import Dummy
from algorithms.hub_spawn import HubSpawn

if TYPE_CHECKING:
    from model.model import DroneModel

def get_strategy_instance(name: str, model: DroneModel):
    if name == "dummy":
        return Dummy(model)
    if name == "hub_spawn":
        return HubSpawn(model)