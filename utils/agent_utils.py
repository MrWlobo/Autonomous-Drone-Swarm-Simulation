import numpy as np
from mesa.discrete_space import Cell
from agents.hub import Hub
from utils.distance import hex_distance

def get_closest_available_hub(cell: Cell, hubs: list[Hub]) -> Hub | None:
    available_hubs = [h for h in hubs if h.capacity > len(h.stored_drones)+len(h.incomming_drones)]
    closest_hub = None
    distance = 10**10
    for hub in available_hubs:
        new_dist = hex_distance(cell, hub.cell)
        if new_dist < distance:
            distance = new_dist
            closest_hub = hub
    return closest_hub
    