from typing import Tuple, Optional

import numpy as np

from hold import Hold


class Side:
    LEFT = 0
    RIGHT = 1


# State interface
class IState:
    def is_possible_state(self) -> bool:
        pass

    def generate_possible_moves(self, global_holds):
        pass


def calculate_angle_between_two_vectors(vector1: np.array, vector2: np.array) -> np.float64:
    product = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
    if product > 1.01 or product < -1.01:
        return 2 * np.pi
    elif product > 1 or product < -1:
        return np.pi
    return np.arccos(product)


def get_intersections(p0: np.array, r0, p1: np.array, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    # Calculate distance between centers using numpy's norm function
    
    if isinstance(p0, Hold):
        p0 = p0.coordinates
        
    if isinstance(p1, Hold):
        p1 = p1.coordinates
    

    x0, y0 = p0[0], p0[1]
    x1, y1 = p1[0], p1[1]
    d = np.sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # non intersecting
    if d > r0 + r1 :
        return None
    # One circle within other
    if d < np.absolute(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0**2 - r1**2 + d**2) / (2*d)
        epsilon = 0.000000001
        if -epsilon < r0**2 - a**2 < 0:
            h = 0
        elif -epsilon > r0**2 - a**2:
            return None
        else:
            h = np.sqrt(r0**2 - a**2)
        x2 = x0 + a * (x1 - x0) / d   
        y2 = y0 + a * (y1 - y0) / d   
        x3 = x2 + h * (y1 - y0) / d     
        y3 = y2 - h * (x1 - x0) / d 

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d
        
        return (np.array([x3, y3]), np.array([x4, y4]))


