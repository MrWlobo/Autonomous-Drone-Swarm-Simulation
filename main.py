import solara
from mesa.experimental.devs import ABMSimulator
from mesa.visualization import SolaraViz, make_plot_component
from model.model import DroneModel
# don't remove the 'unused' Layout import, it is necessary for custom CSS to work 
from visualization.viz import VisualizationComponent, Layout # pylint: disable=unused-import # noqa: F401 

# Create individual plot components
plot_active_drones = make_plot_component(["Total Drones", "Active Drones", "Collisions"],
                                         post_process=lambda ax: ax.set_ylim(bottom=0))
plot_completed_deliveries = make_plot_component(["Completed Deliveries"],
                                                post_process=lambda ax: ax.set_ylim(bottom=0))
plot_avg_delivery_time = make_plot_component(["Avg Delivery Time [minutes]",
                                              "Deliveries Per Minute",
                                              "Time Per Meter [seconds]"],
                                             post_process=lambda ax: ax.set_ylim(bottom=0))
plot_altitude = make_plot_component(["Min Altitude", "Mean Altitude", "Max Altitude"],
                                    post_process=lambda ax: ax.set_ylim(bottom=0))

@solara.component
def PlotsComponent(model: DroneModel):
    # Helper to handle the tuple return type of make_plot_component
    def render(plot_obj):
        if isinstance(plot_obj, tuple):
            component = plot_obj[0]
            return component(model)
        return plot_obj(model)

    with solara.Column():
        if model.show_active_drones_plot:
            render(plot_active_drones)
        if model.show_completed_deliveries_plot:
            render(plot_completed_deliveries)
        if model.show_avg_delivery_time_plot:
            render(plot_avg_delivery_time)
        if model.show_altitude_plot:
            render(plot_altitude)

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
        "values": ['dummy', 'hub_spawn', 'graph_based', "rule_based"],
        "label": "Algorithm",
    },
    "initial_state_setter_name": {
        "type": "Select",
        "value": "hubs",
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
        "value": 4,
        "label": "Number of Packages",
        "min": 1,
        "max": 500,
        "step": 1,
    },
    "num_package_clusters": {
        "type": "SliderInt",
        "value": 5,
        "label": "Number of Package Clusters",
        "min": 1,
        "max": 20,
        "step": 1,
    },
    "num_hubs": {
        "type": "SliderInt",
        "value": 1,
        "label": "Number of Hubs",
        "min": 0,
        "max": 10,
        "step": 1,
    },
    "drone_speed": {
        "type": "SliderInt",
        "value": 20,
        "label": "Drone Speed [m/s]",
        "min": 1,
        "max": 20,
        "step": 1,
    },
    "drone_acceleration": {
        "type": "SliderInt",
        "value": 4,
        "label": "Drone Acceleration [m/s^2]",
        "min": 2,
        "max": 10,
        "step": 1,
    },
    "drone_battery": {
        "type": "SliderInt",
        "value": 14_300_000,
        "label": "Drone Battery [J]",
        "min": 10_000,
        "max": 28_600_000,
        "step": 10_000,
    },
    "drone_charge_rate": {
        "type": "SliderInt",
        "value": 5700,
        "label": "Drone Charge Rate [J/s]",
        "min": 1000,
        "max": 20000,
        "step": 100,
    },
    "save_every": {
        "type": "SliderInt",
        "value": 0,
        "label": "Save CSV Every X Steps",
        "min": 0,
        "max": 100,
        "step": 5,
    },
    "show_gridlines": {
        "type": "Checkbox",
        "value": False,
        "label": "Show Gridlines",
    },
    "show_active_drones_plot": {
        "type": "Checkbox",
        "value": False,
        "label": "Show Active Drones Plot",
    },
    "show_completed_deliveries_plot": {
        "type": "Checkbox",
        "value": False,
        "label": "Show Completed Deliveries Plot",
    },
    "show_avg_delivery_time_plot": {
        "type": "Checkbox",
        "value": False,
        "label": "Show Avg Delivery Time Plot",
    },
    "show_altitude_plot": {
        "type": "Checkbox",
        "value": False,
        "label": "Show Altitude Plot",
    },
}

simulator = ABMSimulator()

initial_model = DroneModel(
    width=model_params['width'],
    height=model_params['height'],
    preset_name=model_params["preset_name"]["value"],
    num_drones=model_params["num_drones"]["value"],
    num_packages=model_params["num_packages"]["value"],
    num_package_clusters=model_params["num_package_clusters"]["value"],
    num_hubs=model_params["num_hubs"]["value"],
    algorithm_name=model_params["algorithm_name"]["value"],
    initial_state_setter_name=model_params["initial_state_setter_name"]["value"], 
    drone_speed=model_params["drone_speed"]["value"],
    drone_acceleration=model_params["drone_acceleration"]["value"],
    drone_battery=model_params["drone_battery"]["value"],
    drone_charge_rate=model_params["drone_charge_rate"]["value"],
    simulator=simulator,
    show_gridlines=model_params["show_gridlines"]["value"],
    save_every=model_params["save_every"]["value"],
    # Map the boolean params to the model attributes
    show_active_drones_plot=model_params["show_active_drones_plot"]["value"],
    show_completed_deliveries_plot=model_params["show_completed_deliveries_plot"]["value"],
    show_avg_delivery_time_plot=model_params["show_avg_delivery_time_plot"]["value"],
    show_altitude_plot=model_params["show_altitude_plot"]["value"],
)

page = SolaraViz(
    model=initial_model,
    components=[VisualizationComponent,
                PlotsComponent,
                ], 
    model_params=model_params,
    name="Autonomous Drone Swarm Simulation",
    simulator=simulator,
)