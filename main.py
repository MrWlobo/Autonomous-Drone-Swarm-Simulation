from mesa.experimental.devs import ABMSimulator
from mesa.visualization import Slider, SolaraViz

from model.model import DroneModel
from visualization.viz import VisualizationComponent, Layout


model_params = {
    "width": 50,
    "height": 50,
    "preset_name": {
        "type": "Select",
        "value": None,
        "values": ["None", 'hangzhou_35806'],
        "label": "Preset",
    },
    "algorithm_name": {
        "type": "Select",
        "value": 'dummy',
        "values": ['dummy', 'hub_spawn'],
        "label": "Algorithm",
    },
    "num_drones": Slider("Number of Drones", 2, 1, 50),
    "num_packages": Slider("Number of Packages", 4, 1, 50),
    "num_obstacles": Slider("Number of Obstacles", 0, 0, 50),
    "num_hubs": Slider("Number of Hubs", 0, 0, 10),
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
    num_obstacles=model_params["num_obstacles"].value,
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