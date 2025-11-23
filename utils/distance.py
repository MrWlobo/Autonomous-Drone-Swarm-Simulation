import numpy as np
from mesa.discrete_space import Cell

def hex_distance(a: Cell, b: Cell):
    """Computes the shortest path (Manhattan) distance between two hex cells.

    Returns:
        float: The distance between cell a and cell b.
    """
    col1, row1 = a.coordinate
    col2, row2 = b.coordinate

    q1 = col1 - (row1 + (row1 & 1)) // 2 
    r1 = row1

    q2 = col2 - (row2 + (row2 & 1)) // 2
    r2 = row2

    dq = q1 - q2
    dr = r1 - r2

    return (np.abs(dq) + np.abs(dq + dr) + np.abs(dr)) / 2