import sys
import os
from matplotlib import pyplot as plt
from matplotlib.collections import PolyCollection
import matplotlib as mpl
import numpy as np
import solara
from mesa.discrete_space import CellAgent

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from agents.drone import Drone
from agents.drop_zone import DropZone
from agents.hub import Hub
from agents.obstacle import Obstacle
from agents.package import Package
from model.model import DroneModel


def VisualizationComponent(model: DroneModel) -> None:
    """Replaces the default SpaceRenderer to fully support HexGrid.

    Args:
        model (DroneModel): The model to visualize.
    """
    fig = plt.Figure()
    ax = fig.add_subplot(111)
    
    all_coords = np.array([cell.coordinate for cell in model.grid])
    
    min_x, max_x = 0, 1
    min_y, max_y = 0, 1
    
    if len(all_coords) > 0:
        qs = all_coords[:, 0]
        rs = all_coords[:, 1]
        
        heights = np.array([model.get_elevation(cell.coordinate) for cell in model.grid])

        cmap = mpl.colormaps['terrain']

        if heights.size > 0 and heights.max() > heights.min():
            norm = mpl.colors.Normalize(vmin=heights.min(), vmax=heights.max())
            cell_facecolors = cmap(norm(heights))
        else:
            cell_facecolors = np.full((len(heights), 4), cmap(0.5))
        
        target_alpha = 0.11
        
        if cell_facecolors.size > 0:
            cell_facecolors[:, 3] = target_alpha
        
        xs, ys = get_screen_coords(qs, rs)
        
        hex_offsets = get_hex_offsets()
        
        centers = np.column_stack([xs, ys])
        verts = centers[:, np.newaxis, :] + hex_offsets[np.newaxis, :, :]
        
        all_verts = verts.reshape(-1, 2)
        min_x, min_y = np.min(all_verts, axis=0)
        max_x, max_y = np.max(all_verts, axis=0)
        
        if model.background is not None and os.path.exists(model.background):
            img = plt.imread(model.background)
            ax.imshow(img, extent=[min_x, max_x, min_y, max_y], zorder=-1, alpha=0.3)
        
        
        if model.show_gridlines:
            collection = PolyCollection(
                verts, 
                facecolors=cell_facecolors, 
                edgecolors='#cccccc', 
                linewidths=0.5,
                zorder=0,
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

    buffer = 1.0 
    ax.set_xlim(min_x - buffer, max_x + buffer)
    ax.set_ylim(min_y - buffer, max_y + buffer)
    
    ax.set_aspect("equal")
    ax.set_axis_off()
    ax.margins(0)

    data_width = (max_x - min_x) + (buffer * 2)
    data_height = (max_y - min_y) + (buffer * 2)
    
    if data_height == 0:
        data_height = 1 
    
    ratio = data_width / data_height
    
    base_height = 8
    fig.set_size_inches(base_height * ratio, base_height)
    
    fig.tight_layout(pad=0)
    
    solara.FigureMatplotlib(fig)


@solara.component
def Layout(children):
    """
    Custom Layout to hide the default Solara top bar.
    """
    return solara.AppLayout(
        children=[
            solara.Style("""
                /* 1. COMPLETELY HIDE THE TOP BAR */
                header.v-app-bar {
                    display: none !important;
                }
                
                .v-content.solara-content-main {
                    padding-top: 0 !important;
                }

                /* 2. RESET MAIN CONTENT AREA 
                   Forces the simulation grid to go to the very top. */
                main.v-main {
                    padding-top: 0px !important;
                }

                /* 3. STRETCH SIDEBAR TO FULL HEIGHT 
                   Forces the sidebar to start at pixel 0 and end at the bottom. */
                nav.v-navigation-drawer {
                    top: 0px !important;
                    height: 100vh !important;  /* 100% of Viewport Height */
                    max-height: 100vh !important;
                    margin-top: 0px !important;
                    z-index: 100 !important;   /* Ensure it sits above other layers */
                }

                /* 4. ENSURE SIDEBAR INTERNAL CONTENT FILLS HEIGHT */
                .v-navigation-drawer__content {
                    height: 100vh !important;
                }
            """)
        ] + children
    )


def get_screen_coords(q: float, r: float) -> tuple[float, float]:
    """
    Converts hexagonal grid coordinates to Cartesian screen coordinates.

    Args:
        q (float): The column index (or horizontal coordinate) of the cell.
        r (float): The row index (or vertical coordinate) of the cell.

    Returns:
        tuple[float, float]: A tuple (x, y) representing the Cartesian 
        center coordinates of the hexagon on the plot.
    """
    width = 1.0
    
    x = (q - (r % 2) / 2.0) * np.sqrt(3) * width
    y = r * 1.5 * width
    
    return x, y


def get_hex_offsets() -> np.ndarray:
    """
    Calculates the (x, y) vertex coordinates for a regular hexagon centered at (0, 0).

    Applies a 30-degree rotation to the vertices to ensure the correct 
    'pointy-topped' orientation for the grid.

    Returns:
        np.ndarray: A (6, 2) array containing the x and y coordinates of the vertices.
    """
    angles = np.linspace(0, 2 * np.pi, 7)[:-1] 
    
    flat_top_angles = angles - np.pi / 6 
    
    return np.column_stack([np.cos(flat_top_angles), np.sin(flat_top_angles)])


def agent_portrayal(agent: CellAgent) -> dict | None:
    """
    Determines the visual style attributes for a given agent instance.

    Args:
        agent (CellAgent): The simulation agent instance to be visualized.

    Returns:
        dict | None: A dictionary of style parameters or None if the agent is invalid.
    """
    if agent is None:
        return None
    
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
        style.update({"color": "green", "marker": "s", "size": 80, "zorder": 1, "alpha": 0.6})
    else:
        style.update({"color": "gray", "size": 20, "alpha": 0})
    
    return style