import numpy as np


class Hold:
    def __init__(self, x, y):
        self.coordinates = np.array([x, y])
        
    # def __init__(self, np_array: np.array):
    #     self.coordinates = np_array
    @property
    def x(self):
        return self.coordinates[0]

    @property
    def y(self):
        return self.coordinates[1]

    def distance_to(self, other_hold):
        return np.linalg.norm(self.coordinates - other_hold.coordinates)
