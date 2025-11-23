import sys
import os
import matplotlib.pyplot as plt
from matplotlib.patches import RegularPolygon
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
    """
    Converts Grid Coords (q,r) to Screen (x,y) for a Rectangular (Offset) Layout.
    """
    width = 1.0

    x = (q - (r % 2) / 2.0) * np.sqrt(3) * width
    y = r * 1.5 * width
    
    return x, y


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
    
    for cell in model.grid:
        q, r = cell.coordinate
        x, y = get_screen_coords(q, r)
        
        hex_patch = RegularPolygon(
            (x, y), 
            numVertices=6, 
            radius=1.0, 
            orientation=np.radians(0), 
            facecolor='none', 
            edgecolor='#cccccc', 
            linewidth=1,
            zorder=0
        )
        ax.add_patch(hex_patch)

    batches = {}

    for agent in model.agents:
        if agent.pos is None: continue
        
        style = agent_portrayal(agent)
        marker = style["marker"]
        
        if marker not in batches:
            batches[marker] = {"x": [], "y": [], "c": [], "s": [], "z": [], "a": []}
        
        q, r = agent.pos
        x, y = get_screen_coords(q, r)
        
        batches[marker]["x"].append(x)
        batches[marker]["y"].append(y)
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
    width=15, height=15,
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