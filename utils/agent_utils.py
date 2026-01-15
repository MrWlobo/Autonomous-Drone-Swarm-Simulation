import numpy as np
from mesa.discrete_space import Cell
from agents.hub import Hub
from utils.distance import hex_distance

def get_closest_available_hub(cell: Cell, hubs: list[Hub]) -> Hub | None:
    available_hubs = [h for h in hubs if h.capacity > len(h.stored_drones)+len(h.incomming_drones)]
    best_hub = None
    drones = 10**10
    for hub in available_hubs:
        new_drones = len(hub.incomming_drones)
        if new_drones < drones:
            drones = new_drones
            best_hub = hub
    return best_hub


def is_hub(agent):
    return isinstance(agent, Hub)