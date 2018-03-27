import numpy as np
from dataclasses import dataclass
from itertools import product
import matplotlib.pyplot as plt


def mat3x3():
    return product(range(-1, 2), range(-1,2))


@dataclass
class Field2DGen(object):
    grid: np.ndarray

    def buildToClosestBoundary(self, condition=lambda v: v >= 0.5):
        """
        vector field representing shortest displace to move to white space
        :return:
        """
        grid = self.grid

        shape = grid.shape
        fieldShape = (*shape, 2)
        field = np.zeros(fieldShape, np.int)

        def isBlack(i, j): # TODO: as a param
            return condition(grid[i, j])

        # initialize open & visited set, open set assuming sparseness
        _open: list = []
        _visited: np.ndarray = np.zeros(shape, np.bool)
        for (i,j), _ in np.ndenumerate(grid):
            if not isBlack(i,j):
                _open.append((i,j))
                _visited[i,j] = True

        def updateOnce():
            nonlocal _open
            nonlocal _visited

            nextOpen = []

            for (i, j) in _open:
                for di, dj in mat3x3():
                    ii = i + di
                    jj = j + dj

                    existingNorm = None
                    if 0 <= ii < np.size(_visited, 0) and 0 <= jj < np.size(_visited, 1):
                        isVisited = _visited[ii, jj]
                        if isVisited:
                            existingVector = field[ii, jj, :]
                            existingNorm = np.linalg.norm(existingVector)

                        vector = field[i, j, :] + [di, dj]

                        # take the smaller one
                        if existingNorm is None:
                            field[ii, jj, :] = vector
                            nextOpen.append((ii, jj))
                            _visited[ii, jj] = True
                        elif np.linalg.norm(vector) < existingNorm:
                            field[ii, jj, :] = vector

            # plot(field, _open)
            _open = nextOpen

        while len(_open) > 0:
            updateOnce()

        return field


def plot(field=None, points=None):
    if field is not None:
        plt.quiver(field[:,:,1], field[:,:,0])
    if points is not None:
        x = list(map(lambda v: v[1], points))
        y = list(map(lambda v: v[0], points))
        plt.scatter(x, y, s=20)
    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.close()
    plt.show()
