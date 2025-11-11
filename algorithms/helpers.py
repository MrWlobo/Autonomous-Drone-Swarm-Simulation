from algorithms.dummy import Dummy
from algorithms.hub_spawn import HubSpawn

def get_strategy_instance(name, model):
    if name == "dummy":
        return Dummy(model)
    if name == "hub_spawn":
        return HubSpawn(model)