import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from agents.drone import Drone
from agents.hub import Hub
from agents.obstacle import Obstacle
from agents.package import Package
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
        portrayal.update(("color", "red"), ("zorder", 2))
    elif isinstance(agent, Hub):
        portrayal.update(("marker", "p"), ("color", "cyan"))
    elif isinstance(agent, Obstacle):
        portrayal.update(("marker", "s"), ("color", "black"))
    elif isinstance(agent, Package):
        portrayal.update(("marker", "*"), ("color", "brown"), ("zorder", 1))

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
model = DroneModel(width=10, height=10, num_drones=1, num_packages=1, algorithm_name='dummy',
                   drone_stats=drone_stats, simulator=simulator)

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