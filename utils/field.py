import numpy as np
from dataclasses import dataclass

# all neighbours
from typing import Callable

from utils import grid2Open

neighbours = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def defaultCondition(v) -> bool:
    return v > 0.5


@dataclass
class FieldGen(object):
    grid: np.ndarray
    condition: Callable = defaultCondition

    def __post_init__(self):
        self.shape = self.grid.shape
        self._nearest = np.zeros((*self.shape, 2), np.int)
        self.coverage = np.zeros((*self.shape, 2), np.int)

        # initialize open & visited set, open set assuming sparseness
        self.openSet: list = grid2Open(self.grid, self.condition)
        self.visitedSet: np.ndarray = self.condition(self.grid)
        self.generated = False


    def nearest(self):
        """
        vector field representing shortest displace to move to white space
        :return:
        """
        if self.generated is True:
            return self._nearest

        def updateOnce(_epoch: int):
            nextOpenSet = list()
            print("epoch", _epoch)
            for (i, j) in self.openSet:
                # print("propagating", i, j)
                for di, dj in neighbours:
                    iTilde = i + di
                    jTilde = j + dj

                    if 0 <= iTilde < self.shape[0] and 0 <= jTilde < self.shape[1]:
                        isVisited = self.visitedSet[iTilde, jTilde]
                        if not isVisited:
                            vector = self._nearest[i, j, :] + [di, dj]

                            self._nearest[iTilde, jTilde, :] = vector

                            nextOpenSet.append((iTilde, jTilde))
                            self.visitedSet[iTilde, jTilde] = True

                            self.coverage[iTilde, jTilde] = _epoch

            # plot(field, _open)
            self.openSet = nextOpenSet

        epoch = 1
        while len(self.openSet) > 0:
            updateOnce(epoch)
            epoch += 1

        self.generated = True
        return self._nearest

    # def repulsion(self):
    #     nearest = self.nearestGrid()
    #
    #     def nearest2repulsion(v: np.array) -> tuple:
    #         dist = dist(v)
    #
    #     nearestTuples = np.apply_along_axis()
