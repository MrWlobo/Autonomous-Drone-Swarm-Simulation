import pytest
from mesa.discrete_space import Cell
from mesa.experimental.devs import ABMSimulator

from algorithms.graph_based import GraphBased
from model.model import DroneModel
from utils.distance import hex_distance


@pytest.fixture
def GraphBasedInstance():
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
            "value": 'graph_based',
            "values": ['dummy', 'hub_spawn', 'graph_based'],
            "label": "Algorithm",
        },
        "initial_state_setter_name": {
            "type": "Select",
            "value": "random",
            "values": ["random", "hubs"],
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
            "value": 40,
            "label": "Number of Packages",
            "min": 1,
            "max": 50,
            "step": 1,
        },
        "num_obstacles": {
            "type": "SliderInt",
            "value": 500,
            "label": "Number of Obstacles",
            "min": 0,
            "max": 500,
            "step": 1,
        },
        "num_hubs": {
            "type": "SliderInt",
            "value": 10,
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
            "value": 4,
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
        "show_gridlines": {
            "type": "Checkbox",
            "value": False,
            "label": "Show Gridlines",
        },
    }

    simulator = ABMSimulator()

    model = DroneModel(
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
    simulator=simulator,
    show_gridlines=model_params["show_gridlines"]["value"],
    )

    graph_based = GraphBased(model)

    return graph_based

def test__create_adjacency_matrix_size(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    assert len(GraphBasedInstance.adjacency_matrix) == len(GraphBasedInstance.model.get_hubs()), "Number of rows should be equal to the number of hubs."
    assert len(GraphBasedInstance.adjacency_matrix[0]) == len(GraphBasedInstance.model.get_packages()) + len(GraphBasedInstance.model.get_hubs()), "Number of columns should be equal to the sum of the numbers of hubs and packages."

def test__direct_distance(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    for i, j in zip(range(20, 40), range(20, 40)):
        for x, y in zip(range(20, 40), range(20, 40)):
            c1 = Cell((i, j), [])
            c2 = Cell((x, y), [])
            assert len(GraphBasedInstance._direct_path(c1, c2)) == hex_distance(c1, c2) + 1
