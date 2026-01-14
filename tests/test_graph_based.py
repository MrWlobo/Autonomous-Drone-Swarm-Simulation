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
    algorithm_name=model_params["algorithm_name"]["value"],
    initial_state_setter_name=model_params["initial_state_setter_name"]["value"],
    drone_speed=model_params["drone_speed"]["value"],
    drone_acceleration=model_params["drone_acceleration"]["value"],
    simulator=simulator,
    show_gridlines=model_params["show_gridlines"]["value"],
    )

    graph_based = GraphBased(model)

    return graph_based

def test__create_adjacency_matrix_size(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    assert len(GraphBasedInstance.adjacency_matrix) == len(GraphBasedInstance.model.get_hubs()) + len(GraphBasedInstance.model.get_packages()), "Number of rows should be equal to the sum of the numbers of hubs and packages."
    assert len(GraphBasedInstance.adjacency_matrix[0]) == len(GraphBasedInstance.model.get_packages()) + len(GraphBasedInstance.model.get_hubs()), "Number of columns should be equal to the sum of the numbers of hubs and packages."

def test__direct_distance(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    for i, j in zip(range(20, 40), range(20, 40)):
        for x, y in zip(range(20, 40), range(20, 40)):
            c1 = Cell((i, j), [])
            c2 = Cell((x, y), [])
            assert len(GraphBasedInstance._direct_path(c1, c2)) == hex_distance(c1, c2) + 1

def test_find_best_path_does_not_crash(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    drones = list(GraphBasedInstance.model.get_drones())
    packages = list(GraphBasedInstance.model.get_packages())

    if not drones or not packages:
        pytest.skip("No drones or packages")

    drone = drones[0]
    drone.package = packages[0]

    path = GraphBasedInstance._find_best_path(drone)
    print(path)
    print(type(path))
    assert isinstance(path, list)

def test_find_best_path_structure(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    drones = list(GraphBasedInstance.model.get_drones())
    packages = list(GraphBasedInstance.model.get_packages())

    if not drones or not packages:
        pytest.skip("No drones or packages")

    drone = drones[0]
    drone.package = packages[0]

    path = GraphBasedInstance._find_best_path(drone)

    for entry in path:
        assert isinstance(entry, tuple), f"Path entry {entry} is not a tuple"
        assert len(entry) == 2, f"Path entry {entry} does not have 2 elements"
        node, battery = entry
        assert battery >= 0, f"Battery value {battery} is negative"
        assert hasattr(node, "cell"), f"Node {node} has no cell attribute"

def test_delivery_implies_return(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    drones = list(GraphBasedInstance.model.get_drones())
    packages = list(GraphBasedInstance.model.get_packages())

    if not drones or not packages:
        pytest.skip("No drones or packages")

    drone = drones[0]
    drone.package = packages[0]

    path = GraphBasedInstance._find_best_path(drone)

    if not path:
        pytest.skip("No feasible path found")

    # Last node must be a hub
    last_node, _ = path[-1]
    assert last_node in GraphBasedInstance.model.get_hubs(), \
        f"Last node {last_node} is not a hub"

    # Drone package appears at most once
    drop_count = sum(1 for node, _ in path if node is drone.package)
    assert drop_count <= 1, f"Package appears {drop_count} times in path"

def test_hub_recharge(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    drones = list(GraphBasedInstance.model.get_drones())
    packages = list(GraphBasedInstance.model.get_packages())

    if not drones or not packages:
        pytest.skip("No drones or packages")

    drone = drones[0]
    drone.package = packages[0]

    path = GraphBasedInstance._find_best_path(drone)

    for (node, battery), (next_node, next_battery) in zip(path, path[1:]):
        if node in GraphBasedInstance.model.get_hubs():
            assert next_battery >= battery, f"Battery did not increase at hub {node}"
        else:
            assert next_battery <= battery, f"Battery increased outside hub at {node}"

def test_battery_never_drops_below_safe(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    drones = list(GraphBasedInstance.model.get_drones())
    packages = list(GraphBasedInstance.model.get_packages())

    if not drones or not packages:
        pytest.skip("No drones or packages")

    drone = drones[0]
    drone.package = packages[0]

    path = GraphBasedInstance._find_best_path(drone)
    if not path:
        pytest.skip("No feasible path found")

    safe_level = GraphBasedInstance.model.drone_stats.drone_safe_battery_level

    for node, battery in filter(lambda x: x[1] != 0, path):
        assert battery >= safe_level, f"Battery below safe level at {node}: {battery}"

def test_no_path_when_package_unreachable(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    drones = list(GraphBasedInstance.model.get_drones())
    packages = list(GraphBasedInstance.model.get_packages())

    if not drones or not packages:
        pytest.skip("No drones or packages")

    drone = drones[0]
    drone.package = packages[0]

    # Artificially set drone battery to very low
    drone.battery = 1

    path = GraphBasedInstance._find_best_path(drone)
    assert path == [], "Path should be empty if drone cannot reach drop zone safely"

def test_path_ends_at_nearest_hub(GraphBasedInstance):
    GraphBasedInstance._create_adjacency_matrix()
    drones = list(GraphBasedInstance.model.get_drones())
    packages = list(GraphBasedInstance.model.get_packages())
    hubs = list(GraphBasedInstance.model.get_hubs())

    if not drones or not packages or not hubs:
        pytest.skip("No drones, packages, or hubs")

    drone = drones[0]
    drone.package = packages[0]

    path = GraphBasedInstance._find_best_path(drone)
    if not path:
        pytest.skip("No feasible path found")

    last_node, _ = path[-1]
    drop_idx = len(GraphBasedInstance.hub_list) + GraphBasedInstance.drop_zone_list.index(drone.package.drop_zone)

    min_cost_idx = min(
        range(len(GraphBasedInstance.hub_list)),
        key=lambda h_idx: GraphBasedInstance._estimated_cost(drone, *GraphBasedInstance.adjacency_matrix[drop_idx][h_idx])
    )

    assert last_node == GraphBasedInstance.hub_list[min_cost_idx], \
        f"Last node {last_node} is not the minimal energy hub according to adjacency matrix"
