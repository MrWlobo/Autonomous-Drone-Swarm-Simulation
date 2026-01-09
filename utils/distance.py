import numpy as np
from mesa.discrete_space import Cell

def hex_distance(a: Cell, b: Cell):
    """Computes the shortest path (Manhattan) distance between two hex cells.

    Returns:
        int: The distance between cell a and cell b.
    """
    col1, row1 = a.coordinate
    col2, row2 = b.coordinate

    q1 = col1 - (row1 + (row1 & 1)) // 2 
    r1 = row1

    q2 = col2 - (row2 + (row2 & 1)) // 2
    r2 = row2

    dq = q1 - q2
    dr = r1 - r2

    return round((np.abs(dq) + np.abs(dq + dr) + np.abs(dr)) / 2)

def qrs_hex_distance(qrs1: tuple[int, int, int], qrs2: tuple[int, int, int]) -> int:
    q1, r1, s1 = qrs1
    q2, r2, s2 = qrs2
    return max(abs(q1-q2), abs(r1-r2), abs(s1-s2))

def xy_to_qrs(col: int | tuple[int, int], row: int | None = None) -> tuple[int, int, int]:
    """ Converts Cartesian coordinates to cubic coordinates for hex grid calculations
    Accepts:
    1. xy_to_qrs(col, row)
    2. xy_to_qrs((col, row))
    Returns: tuple[int, int, int]
    """
    col, row = col if isinstance(col, tuple) else (col, row)
    q = col - (row + (row & 1)) // 2 
    r = row
    s = - q - r
    return (q, r, s)

def qrs_to_xy(q: int | tuple[int, int, int], r: int | None = None, s: int | None = None) -> tuple[int, int]:
    """ Converts cubic coordinates to Cartesian coordinates for hex grid calculations
    Accepts:
    1. qrs_to_xy(q, r, s)
    2. qrs_to_xy((q, r, s))
    Returns: tuple[int, int]
    """
    q, r, s = q if isinstance(q, tuple) else (q, r, s)
    col = q + (r + (r & 1)) // 2
    row = r
    return (col, row)

def hex_vector(cell1: Cell, cell2: Cell) -> tuple[int, int, int]:
    """Computes the vector between two hex cells.
    Returns:
        tuple[int, int, int]: The vector between cell1 and cell2.
    """
    q1, r1, s1 = xy_to_qrs(cell1.coordinate[0], cell1.coordinate[1])
    q2, r2, s2 = xy_to_qrs(cell2.coordinate[0], cell2.coordinate[1])
    return (q2 - q1, r2 - r1, s2 - s1)

def hex_vector_len(hex_vect: tuple[int, int, int]):
    return max(abs(hex_vect[0]), abs(hex_vect[1]), abs(hex_vect[2]))

def normalize_hex_vector(vector: tuple[int, int, int], n: int) -> tuple[int, int, int]:
    dq, dr, ds = vector
    length = hex_vector_len((dq, dr, ds))
    if length == 0:
        return (0, 0, 0)
    k = n / length
    q_prime = dq * k
    r_prime = dr * k
    s_prime = ds * k
    q_round = round(q_prime)
    r_round = round(r_prime)
    s_round = round(s_prime)
    err_q = abs(q_round - q_prime)
    err_r = abs(r_round - r_prime)
    err_s = abs(s_round - s_prime)
    current_sum = q_round + r_round + s_round
    if current_sum == 1:
        if err_q > err_r and err_q > err_s:
            q_round -= 1
        elif err_r > err_s:
            r_round -= 1
        else:
            s_round -= 1
    elif current_sum == -1:
        if err_q > err_r and err_q > err_s:
            q_round += 1
        elif err_r > err_s:
            r_round += 1
        else:
            s_round += 1
    return (int(q_round), int(r_round), int(s_round))

def add_hex_vectors(vector1: tuple[int, int, int], vector2: tuple[int, int, int]) -> tuple[int, int, int]:
    return (vector1[0] + vector2[0], vector1[1] + vector2[1], vector1[2] + vector2[2])

def sub_hex_vectors(vector1: tuple[int, int, int], vector2: tuple[int, int, int]) -> tuple[int, int, int]:
    return (vector1[0] - vector2[0], vector1[1] - vector2[1], vector1[2] - vector2[2])

def reverse_hex_vector(vector: tuple[int, int, int]) -> tuple[int, int, int]:
    return (-vector[0], -vector[1], -vector[2])

def divide_hex_vector(vector: tuple[int, int, int], divider) -> tuple[int, int, int]:
    return (vector[0]/divider, vector[1]/divider, vector[2]/divider)

def round_hex_vector(vector: tuple[float, float, float]) -> tuple[int, int, int]:
    return (round(vector[0]), round(vector[1]), round(vector[2]))