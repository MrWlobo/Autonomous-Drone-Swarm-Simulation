import sys
import os
from matplotlib import pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
import solara

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from model.model import DroneModel
from agents.drone import Drone
from agents.drop_zone import DropZone
from agents.hub import Hub
from agents.obstacle import Obstacle
from agents.package import Package


def VisualizationComponent(model):
    fig = plt.Figure(figsize=(10, 8))
    ax = fig.add_subplot(111)
    
    all_coords = np.array([cell.coordinate for cell in model.grid])
    
    if len(all_coords) > 0:
        qs = all_coords[:, 0]
        rs = all_coords[:, 1]
        
        xs, ys = get_screen_coords(qs, rs)
        
        hex_offsets = get_hex_offsets()
        
        centers = np.column_stack([xs, ys])
        verts = centers[:, np.newaxis, :] + hex_offsets[np.newaxis, :, :]
        
        collection = PolyCollection(
            verts, 
            facecolors='none', 
            edgecolors='#cccccc', 
            linewidths=0.5,
            zorder=0,
        )
        ax.add_collection(collection)

    batches = {}

    for agent in model.agents:
        if agent.pos is None: continue
        
        style = agent_portrayal(agent)
        if style is None: 
            continue
        
        marker = style["marker"]
        
        if marker not in batches:
            batches[marker] = {"x": [], "y": [], "c": [], "s": [], "z": [], "a": []}
        
        q, r = agent.pos
        x_a, y_a = get_screen_coords(q, r) 
        
        batches[marker]["x"].append(x_a)
        batches[marker]["y"].append(y_a)
        batches[marker]["c"].append(style["color"])
        batches[marker]["s"].append(style["size"])
        batches[marker]["z"].append(style["zorder"])
        batches[marker]["a"].append(style["alpha"])

    for marker, data in batches.items():
        ax.scatter(
            data["x"], data["y"], 
            c=data["c"], s=data["s"], 
            marker=marker, 
            zorder=np.mean(data["z"]), 
            alpha=data["a"]
        )

    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    ax.set_aspect("equal")
    ax.set_axis_off()
    
    ax.autoscale(enable=True)
    
    solara.FigureMatplotlib(fig)


def get_screen_coords(q, r):
    width = 1.0
    
    x = (q - (r % 2) / 2.0) * np.sqrt(3) * width
    y = r * 1.5 * width
    return x, y


def get_hex_offsets():
    """
    Generates the (x, y) offsets for a single hexagon vertices 
    relative to its center (0,0).
    """
    angles = np.linspace(0, 2 * np.pi, 7)[:-1] 
    
    flat_top_angles = angles - np.pi / 6 
    
    return np.column_stack([np.cos(flat_top_angles), np.sin(flat_top_angles)])


def agent_portrayal(agent):
    if agent is None: return None
    
    style = {"size": 50, "marker": "o", "zorder": 2, "alpha": 1.0}
    
    if isinstance(agent, Drone):
        style.update({"color": "red", "size": 100, "zorder": 10})
    elif isinstance(agent, Hub):
        style.update({"color": "cyan", "marker": "p", "size": 150, "zorder": 4})
    elif isinstance(agent, Obstacle):
        style.update({"color": "black", "marker": "s", "size": 100})
    elif isinstance(agent, Package):
        style.update({"color": "brown", "marker": "*", "size": 80, "zorder": 5})
    elif isinstance(agent, DropZone):
        style.update({"color": "green", "marker": "s", "size": 80, "zorder": 1, "alpha": 0.3})
    else:
        style.update({"color": "gray", "size": 20, "alpha": 0})
    
    return style
