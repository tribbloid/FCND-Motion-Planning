import numpy as np
from typing import List

from planning_utils import create_grid


def loadGrid(safetyDistance=5, targetAltitude=3):
    # Read in obstacle map
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
    # Determine offsets between grid and map
    north_offset = int(np.abs(np.min(data[:, 0])))
    east_offset = int(np.abs(np.min(data[:, 1])))
    print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
    # Define a grid for a particular altitude and safety margin around obstacles
    grid = create_grid(data, targetAltitude, safetyDistance)
    return grid, east_offset, north_offset


def grid2Open(grid, condition) -> List[tuple]:
    ii = np.argwhere(condition(grid))

    def toTuple(vs) -> tuple:
        return tuple(vs)

    result = list(map(toTuple, ii))
    return result
