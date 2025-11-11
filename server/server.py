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
model = DroneModel(width=15, height=15, num_drones=2, num_packages=4, num_hubs=5,
                   algorithm_name='hub_spawn', drone_stats=drone_stats, simulator=simulator)

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