import sys
import os
import matplotlib.pyplot as plt
from matplotlib.patches import RegularPolygon
from matplotlib.collections import PolyCollection
import solara
import numpy as np
from mesa.experimental.devs import ABMSimulator
from mesa.visualization import Slider, SolaraViz

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from model.model import DroneModel
from agents.drone import Drone
from agents.drop_zone import DropZone
from agents.hub import Hub
from agents.obstacle import Obstacle
from agents.package import Package


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
    angles = np.linspace(0, 2*np.pi, 7)[:-1] # 6 angles
    return np.column_stack([np.cos(angles), np.sin(angles)])


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
            zorder=0
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


model_params = {
    "width": 50,
    "height": 50,
    "algorithm_name": {
        "type": "Select",
        "value": 'dummy',
        "values": ['dummy', 'hub_spawn'],
        "label": "Algorithm",
    },
    "num_drones": Slider("Number of Drones", 2, 1, 50),
    "num_packages": Slider("Number of Packages", 4, 1, 50),
    "num_hubs": Slider("Number of Hubs", 5, 1, 10),
    "drone_speed": Slider("Drone Speed", 1, 1, 5),
    "drone_battery": Slider("Drone Battery", 1, 1, 5),
    "drain_rate": Slider("Drone Battery Drain Rate", 1, 1, 5),
}

simulator = ABMSimulator()

initial_model = DroneModel(
    width=model_params['width'],
    height=model_params['height'],
    num_drones=model_params["num_drones"].value,
    num_packages=model_params["num_packages"].value,
    num_hubs=model_params["num_hubs"].value,
    algorithm_name=model_params["algorithm_name"]["value"],
    drone_speed=model_params["drone_speed"].value,
    drone_battery=model_params["drone_battery"].value,
    drain_rate=model_params["drain_rate"].value,
    simulator=simulator,
)

page = SolaraViz(
    model=initial_model,
    components=[VisualizationComponent], 
    model_params=model_params,
    name="Autonomous Drone Swarm Simulation",
    simulator=simulator,
)