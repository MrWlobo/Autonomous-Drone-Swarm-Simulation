import sys, os


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from agents.drone import Drone
from agents.hub import Hub
from agents.obstacle import Obstacle
from agents.package import Package
from agents.drop_zone import DropZone
from model.model import DroneModel, DroneStats
from mesa.experimental.devs import ABMSimulator
from mesa.visualization import (
    CommandConsole,
    Slider,
    SolaraViz,
    SpaceRenderer,
    make_plot_component,
)
import solara
from mesa.visualization.components import AgentPortrayalStyle

def drone_portrayal(agent):
    if agent is None:
        return

    portrayal = AgentPortrayalStyle(
        size=50,
        marker="o",
        zorder=3,
    )

    if isinstance(agent, Drone):
        portrayal.update(("color", "red"), ("zorder", 10))
    elif isinstance(agent, Hub):
        portrayal.update(("marker", "p"), ("color", "cyan"), ("zorder", 4))
    elif isinstance(agent, Obstacle):
        portrayal.update(("marker", "s"), ("color", "black"))
    elif isinstance(agent, Package):
        portrayal.update(("marker", "*"), ("color", "brown"), ("zorder", 3))
    elif isinstance(agent, DropZone):
        portrayal.update(("marker", "o"), ("color", "green"), ("zorder", 2))

    return portrayal

model_params = {
    "algorithm_name": {
        "type": "Select",
        "value": 'hub_spawn',
        "values": ['hub_spawn', 'dummy'],
        "label": "Algorithm",
    },
    "num_drones": Slider("Number of Drones", 2, 1, 50),
    "num_packages": Slider("Number of Packages", 4, 1, 50),
    "num_hubs": Slider("Number of Hubs", 5, 1, 10),
    "drone_speed": Slider("Drone Speed", 1, 1, 5),
    "drone_battery": Slider("Drone Battery", 1, 1, 5),
    "drain_rate": Slider("Drone Battery Drain Rate", 1, 1, 5),
}

def post_process_space(ax):
    ax.set_aspect("equal")
    ax.set_xticks(range(model.width + 1))
    ax.set_yticks(range(model.height + 1))
    ax.grid(True)
    ax.set_xlim(-0.5, model.width + 0.5) 
    ax.set_ylim(-0.5, model.height + 0.5)


simulator = ABMSimulator()
drone_stats = DroneStats(1,1,0)
model = DroneModel(
    width=15, 
    height=15,
    num_drones=model_params["num_drones"].value,
    num_packages=model_params["num_packages"].value,
    num_hubs=model_params["num_hubs"].value,
    algorithm_name=model_params["algorithm_name"]["value"],
    drone_speed=model_params["drone_speed"].value,
    drone_battery=model_params["drone_battery"].value,
    drain_rate=model_params["drain_rate"].value,
    simulator=simulator
)

renderer = SpaceRenderer(
    model,
    backend="matplotlib",
)
renderer.draw_agents(drone_portrayal)
renderer.post_process = post_process_space


page = SolaraViz(
    model,
    renderer,
    components=[],
    model_params=model_params,
    name="Autonomous Drone Swarm Simulation",
    simulator=simulator,
)
page  # noqa