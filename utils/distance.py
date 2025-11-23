import numpy as np
from mesa.discrete_space import Cell

def hex_distance(a: Cell, b: Cell):
        """
        Calculates distance between two cells.
        1. Converts Odd-R Offset coordinates (col, row) to Axial (q, r).
        2. Calculates Manhattan distance on the Axial/Cube plane.
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