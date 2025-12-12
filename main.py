from mesa.experimental.devs import ABMSimulator
from mesa.visualization import SolaraViz
from model.model import DroneModel
# don't remove the 'unused' Layout import, it is necessary for custom CSS to work 
from visualization.viz import VisualizationComponent, Layout # pylint: disable=unused-import # noqa: F401 


model_params = {
    "width": 100,
    "height": 100,
    "preset_name": {
        "type": "Select",
        "value": "None",
        "values": ["None", 'hangzhou_35806', 'shanghai_56909', 'yantai_31702', 'chongqing_38774'],
        "label": "Preset",
    },
    "algorithm_name": {
        "type": "Select",
        "value": 'dummy',
        "values": ['dummy', 'hub_spawn'],
        "label": "Algorithm",
    },
    "initial_state_setter_name": {
        "type": "Select",
        "value": "random",
        "values": ["random"],
        "label": "Initial State",
    },
    "num_drones": {
        "type": "SliderInt",
        "value": 2,
        "label": "Number of Drones",
        "min": 1,
        "max": 50,
        "step": 1,
    },
    "num_packages": {
        "type": "SliderInt",
        "value": 4,
        "label": "Number of Packages",
        "min": 1,
        "max": 50,
        "step": 1,
    },
    "num_obstacles": {
        "type": "SliderInt",
        "value": 0,
        "label": "Number of Obstacles",
        "min": 0,
        "max": 50,
        "step": 1,
    },
    "num_hubs": {
        "type": "SliderInt",
        "value": 0,
        "label": "Number of Hubs",
        "min": 0,
        "max": 10,
        "step": 1,
    },
    "drone_speed": {
        "type": "SliderInt",
        "value": 20,
        "label": "Drone Speed",
        "min": 1,
        "max": 20,
        "step": 1,
    },
    "drone_acceleration": {
        "type": "SliderInt",
        "value": 2,
        "label": "Drone Acceleration",
        "min": 2,
        "max": 10,
        "step": 1,
    },
    "drone_battery": {
        "type": "SliderInt",
        "value": 100,
        "label": "Drone Battery",
        "min": 1,
        "max": 100,
        "step": 1,
    },
    "drain_rate": {
        "type": "SliderInt",
        "value": 1,
        "label": "Drone Battery Drain Rate",
        "min": 0,
        "max": 5,
        "step": 1,
    },
}

simulator = ABMSimulator()

initial_model = DroneModel(
    width=model_params['width'],
    height=model_params['height'],
    preset_name=model_params["preset_name"]["value"],
    num_drones=model_params["num_drones"]["value"],
    num_packages=model_params["num_packages"]["value"],
    num_hubs=model_params["num_hubs"]["value"],
    num_obstacles=model_params["num_obstacles"]["value"],
    algorithm_name=model_params["algorithm_name"]["value"],
    initial_state_setter_name=model_params["initial_state_setter_name"]["value"], 
    drone_speed=model_params["drone_speed"]["value"],
    drone_acceleration=model_params["drone_acceleration"]["value"],
    drone_battery=model_params["drone_battery"]["value"],
    drain_rate=model_params["drain_rate"]["value"],
    simulator=simulator,
)

page = SolaraViz(
    model=initial_model,
    components=[VisualizationComponent], 
    model_params=model_params,
    name="Autonomous Drone Swarm Simulation",
    simulator=simulator,
)