import numpy as np
from dataclasses import dataclass

# all neighbours
from utils import grid2Open

neighbours = [(1, 0), (0, 1), (-1, 0), (0, -1)]


@dataclass
class Generate2DField(object):
    grid: np.ndarray

    def __post_init__(self):
        self.shape = self.grid.shape
        self.field = np.zeros((*self.shape, 2), np.int)
        self.coverage = np.zeros((*self.shape, 2), np.int)

        # initialize open & visited set, open set assuming sparseness
        self.openSet: list = grid2Open(self.grid, self.condition)
        self.visitedSet: np.ndarray = self.condition(self.grid)

    @staticmethod
    def condition(v) -> bool:
        return v > 0.5

    def nearestGrid(self):
        """
        vector field representing shortest displace to move to white space
        :return:
        """

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
                            vector = self.field[i, j, :] + [di, dj]

                            self.field[iTilde, jTilde, :] = vector

                            nextOpenSet.append((iTilde, jTilde))
                            self.visitedSet[iTilde, jTilde] = True

                            self.coverage[iTilde, jTilde] = _epoch

            # plot(field, _open)
            self.openSet = nextOpenSet

        epoch = 1
        while len(self.openSet) > 0:
            updateOnce(epoch)
            epoch += 1

        return self.field

    # def repulsion(self):
    #     nearest = self.nearestGrid()
    #
    #     def nearest2repulsion(v: np.array) -> tuple:
    #         dist = dist(v)
    #
    #     nearestTuples = np.apply_along_axis()
